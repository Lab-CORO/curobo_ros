#!/usr/bin/env python3
"""
Grasp MPC Planner - MPC closed-loop control for reactive grasping.

Inspired by cuRobo's MotionGen.plan_grasp() (motion_gen.py:4198) and
MotionGen.attach_objects_to_robot() / attach_spheres_to_robot() (motion_gen.py:2327+),
and the block-stacking example (curobo.org/advanced_examples/2_block_stacking_example.html).

Key design choices:
  - link6 collision disabled when EE enters the final approach zone
    (same principle as plan_grasp's disable_collision_links during linear phase)
  - Object attachment after grasp uses camera voxels directly from VRAM:
    voxels within a fixed radius of link6 are transformed into the link6
    frame and attached as spheres to the "attached_object" link
    (same technique as attach_spheres_to_robot / KinematicsTensorConfig.attach_object)

Grasp Sequence:
  1. APPROACH : MPC to pre-grasp pose (world +Z offset, full collision)
  2. GRASP    : MPC to grasp pose; link6 disabled near goal
  3. ATTACH   : Voxels around link6 attached as robot spheres; object removed from world
  4. RETREAT  : MPC to post-grasp pose (object now moves with robot)
  5. Close gripper
"""

import time
import traceback
from enum import Enum
from typing import Optional

import torch
from curobo.types.math import Pose
from curobo.types.robot import JointState
from curobo_msgs.action import SendTrajectory
from curobo.rollout.rollout_base import Goal

from .mpc_planner import MPCPlanner
from .trajectory_planner import PlannerResult


class GraspState(Enum):
    IDLE     = 0
    APPROACH = 1
    GRASP    = 2
    ATTACHED = 3
    RETREAT  = 4


class GraspMPCPlanner(MPCPlanner):
    """
    MPC-based reactive grasp planner.

    Extends MPCPlanner with a four-phase grasp sequence:
      APPROACH → GRASP (link6 disabled near goal) → ATTACH → RETREAT

    The ATTACH step reads occupied voxels directly from VRAM
    (world_coll_checker voxel tensors), filters those within
    `grasp_attach_radius` of link6, transforms them into the link6 frame,
    and writes them as collision spheres to the "attached_object" link in
    the MPC kinematics config.

    Prerequisites in the robot YAML:
      The robot must have an "attached_object" link pre-allocated with
      enough spheres (≥ grasp_attach_max_spheres). Without it, attach
      is silently skipped.

    ROS parameters (all declared in __init__):
      grasp_pre_grasp_offset_x           float  pre-grasp x offset in world frame   (0.10 m)
      grasp_post_grasp_offset_x          float  post-grasp x offset in world frame  (0.15 m)
      grasp_approach_convergence         float  APPROACH convergence threshold       (0.02 m)
      grasp_grasp_convergence            float  GRASP convergence threshold          (0.005 m)
      grasp_retreat_convergence          float  RETREAT convergence threshold        (0.02 m)
      grasp_collision_disable_distance   float  disable link6 when error < this     (0.03 m)
      grasp_attach_radius                float  voxel search radius around link6    (0.10 m)
      grasp_attach_max_spheres           int    max spheres to attach               (50)
      grasp_gripper_close_time           float  sleep if no gripper interface       (1.0 s)
    """

    GRASP_LINK    = "link6"
    ATTACHED_LINK = "attached_object"

    def __init__(self, node, config_wrapper):
        super().__init__(node, config_wrapper)

        self.grasp_state    = GraspState.IDLE
        self.target_pose    = None
        self.pre_grasp_pose = None
        self.post_grasp_pose = None
        self.gripper        = None
        self._link6_disabled = False

        _params = [
            ('grasp_pre_grasp_offset_x',          -0.10), # c'est pas en Z, c'est en X (z c'est le tcp)
            ('grasp_post_grasp_offset_x',          0.15),
            ('grasp_approach_convergence',         0.1),
            ('grasp_grasp_convergence',            0.2),
            ('grasp_retreat_convergence',          0.2),
            ('grasp_collision_disable_distance',   0.3),
            ('grasp_attach_radius',                0.10),
            ('grasp_attach_max_spheres',           50),
            ('grasp_gripper_close_time',           1.0),
        ]
        for name, default in _params:
            try:
                node.declare_parameter(name, default)
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Identity
    # ------------------------------------------------------------------

    def get_planner_name(self) -> str:
        return "Grasp MPC (Reactive Grasping)"

    def get_config_parameters(self) -> list:
        return super().get_config_parameters() + [
            'grasp_pre_grasp_offset_x',
            'grasp_post_grasp_offset_x',
            'grasp_approach_convergence',
            'grasp_grasp_convergence',
            'grasp_retreat_convergence',
            'grasp_collision_disable_distance',
            'grasp_attach_radius',
            'grasp_attach_max_spheres',
            'grasp_gripper_close_time',
        ]

    def set_gripper_interface(self, gripper):
        self.gripper = gripper

    # ------------------------------------------------------------------
    # Planning
    # ------------------------------------------------------------------

    def plan(self, start_state: JointState, goal_request, config: dict,
             robot_context=None) -> PlannerResult:
        """
        Initialise MPC for the APPROACH phase.

        goal_request.target_pose : grasp pose (geometry_msgs/Pose)
        """
        if self.mpc is None:
            return PlannerResult(success=False, message="MPC solver not initialised.")

        try:
            tp = goal_request.target_pose
            self.target_pose = Pose.from_list([
                tp.position.x, tp.position.y, tp.position.z,
                tp.orientation.w, tp.orientation.x, tp.orientation.y,
                tp.orientation.z, 
            ])

            pre_x = self._param('grasp_pre_grasp_offset_x')
            post_x = self._param('grasp_post_grasp_offset_x')
            self.pre_grasp_pose  = self._offset_world_x(self.target_pose,  pre_x)
            self.post_grasp_pose = self._offset_world_x(self.target_pose,  post_x)

            self.grasp_state = GraspState.APPROACH
            self.start_state = start_state
            goal = Goal(current_state=start_state, goal_pose=self.pre_grasp_pose)
            self.goal_buffer = self.mpc.setup_solve_single(goal, 1)
            self.mpc.update_goal(self.goal_buffer)
            self.is_goal_active = True

            self.node.get_logger().info(
                f"Grasp MPC planned: pre_z={pre_x:.3f}m, post_z={post_x:.3f}m, "
                f"approach={self._param('grasp_approach_convergence'):.3f}m, "
                f"grasp={self._param('grasp_grasp_convergence'):.4f}m, "
                f"retreat={self._param('grasp_retreat_convergence'):.3f}m, "
                f"disable_link6_at={self._param('grasp_collision_disable_distance'):.3f}m"
            )

            return PlannerResult(success=True, message="Grasp MPC goal buffer initialised")

        except Exception as e:
            self.node.get_logger().error(f"Grasp MPC plan error: {e}")
            self.node.get_logger().error(traceback.format_exc())
            return PlannerResult(success=False, message=f"Grasp MPC plan error: {e}")

    # ------------------------------------------------------------------
    # Execution
    # ------------------------------------------------------------------

    def execute(self, robot_context, goal_handle=None) -> bool:
        """Execute: APPROACH → GRASP → ATTACH → RETREAT → close gripper."""
        if self.goal_buffer is None:
            self.node.get_logger().error("Grasp MPC not initialised.")
            return False

        try:
            # ── Phase 1: APPROACH ────────────────────────────────────────
            self.node.get_logger().info("Grasp 1/4: APPROACH")
            self.grasp_state = GraspState.APPROACH
            if not self._mpc_phase(
                robot_context, self.pre_grasp_pose,
                self._param('grasp_approach_convergence'),
                goal_handle, "APPROACH"
            ):
                self.node.get_logger().error("APPROACH failed")
                return False

            # ── Phase 2: GRASP (link6 disabled near goal) ────────────────
            self.node.get_logger().info("Grasp 2/4: GRASP")
            self.grasp_state = GraspState.GRASP
            self.update_goal_pose(self.target_pose)
            if not self._mpc_phase(
                robot_context, self.target_pose,
                self._param('grasp_grasp_convergence'),
                goal_handle, "GRASP",
                disable_link_near_goal=True
            ):
                self._ensure_link6_enabled()
                self.node.get_logger().error("GRASP failed")
                return False
            self._ensure_link6_enabled()

            # ── Phase 3: ATTACH ───────────────────────────────────────────
            self.node.get_logger().info("Grasp 3/4: ATTACH")
            self.grasp_state = GraspState.ATTACHED
            current_joint_pose = robot_context.get_joint_pose()
            current_state = JointState.from_position(
                torch.Tensor([current_joint_pose]).to(device=self.node.tensor_args.device)
            )
            attached = self._attach_voxels_to_robot(current_state)
            if not attached:
                self.node.get_logger().warn(
                    "Attach skipped (no voxels or no 'attached_object' link) — "
                    "collision checking of grasped object will be absent"
                )

            # ── Phase 4: RETREAT ──────────────────────────────────────────
            self.node.get_logger().info("Grasp 4/4: RETREAT")
            self.grasp_state = GraspState.RETREAT
            self.update_goal_pose(self.post_grasp_pose)
            if not self._mpc_phase(
                robot_context, self.post_grasp_pose,
                self._param('grasp_retreat_convergence'),
                goal_handle, "RETREAT"
            ):
                self.node.get_logger().error("RETREAT failed")
                return False

            # ── Close gripper ─────────────────────────────────────────────
            self.node.get_logger().info("Closing gripper")
            if not self._close_gripper():
                self.node.get_logger().warn("Gripper close failed")

            self.node.get_logger().info("Grasp sequence completed successfully")
            return True

        except Exception as e:
            self._ensure_link6_enabled()
            self.node.get_logger().error(f"Grasp execute error: {e}")
            self.node.get_logger().error(traceback.format_exc())
            robot_context.stop_robot()
            return False

    # ------------------------------------------------------------------
    # MPC phase loop
    # ------------------------------------------------------------------

    def _mpc_phase(
        self,
        robot_context,
        goal_pose: Pose,
        convergence_threshold: float,
        goal_handle=None,
        phase_name: str = "PHASE",
        disable_link_near_goal: bool = False,
    ) -> bool:
        """
        Run one MPC closed-loop phase until convergence or timeout.

        When disable_link_near_goal=True, link6 collision spheres are disabled
        once pose_error drops below grasp_collision_disable_distance, allowing
        the gripper to make contact with the object (same principle as
        plan_grasp's toggle_link_collision during the linear approach).
        """
        disable_dist = self._param('grasp_collision_disable_distance')
        converged    = False
        tstep        = 0
        self.mpc_time = []
        LOG_EVERY    = 50  # log debug info every N steps

        # Log goal pose for reference
        goal_pos = goal_pose.position.squeeze().tolist()
        self.node.get_logger().info(
            f"{phase_name}: started (threshold={convergence_threshold:.4f}m, "
            f"goal_pos={[f'{v:.3f}' for v in goal_pos]})"
        )

        while not converged and self.is_goal_active:

            if goal_handle is not None and not goal_handle.is_active:
                self.node.get_logger().warn(f"{phase_name}: cancelled")
                robot_context.stop_robot()
                return False

            # Closed-loop state feedback
            joint_pose    = robot_context.get_joint_pose()
            current_state = JointState.from_position(
                torch.Tensor([joint_pose]).to(device=self.node.tensor_args.device)
            )

            t0     = time.time()
            result = self.mpc.step(current_state, 1)
            torch.cuda.synchronize()

            if tstep > 5:
                self.mpc_time.append(time.time() - t0)

            pose_error = result.metrics.pose_error.item()

            # ── Periodic debug log ────────────────────────────────────────
            if tstep % LOG_EVERY == 0:
                # Compute actual EE position via kinematics for comparison
                kin = self.mpc.compute_kinematics(current_state)
                ee_pos = kin.ee_pose.position.squeeze().tolist()
                self.node.get_logger().info(
                    f"{phase_name} step={tstep:4d} | "
                    f"pose_error={pose_error:.4f}m | "
                    f"ee_pos={[f'{v:.3f}' for v in ee_pos]} | "
                    f"goal_pos={[f'{v:.3f}' for v in goal_pos]} | "
                    f"joints={[f'{v:.3f}' for v in joint_pose]}"
                )
            # ─────────────────────────────────────────────────────────────

            # Disable link6 when entering final-approach zone
            if disable_link_near_goal and not self._link6_disabled:
                if pose_error < disable_dist:
                    self._toggle_link6(False)
                    self.node.get_logger().info(
                        f"{phase_name}: {self.GRASP_LINK} collision disabled "
                        f"(error={pose_error:.4f}m)"
                    )

            self._send_mpc_command(robot_context, result.action)

            if goal_handle is not None and tstep % 10 == 0:
                msg = SendTrajectory.Feedback()
                msg.step_progression = 1.0 - min(pose_error / 0.1, 1.0)
                goal_handle.publish_feedback(msg)

            if pose_error < convergence_threshold:
                converged = True
                self.node.get_logger().info(
                    f"{phase_name}: converged at step {tstep} "
                    f"(error={pose_error:.4f}m)"
                )

            tstep += 1
            if tstep > self.max_iterations:
                self.node.get_logger().warn(
                    f"{phase_name}: max iterations ({self.max_iterations}) reached — "
                    f"final pose_error={pose_error:.4f}m "
                    f"(threshold={convergence_threshold:.4f}m)"
                )
                break

        robot_context.stop_robot()

        if self.mpc_time:
            avg = sum(self.mpc_time) / len(self.mpc_time)
            self.node.get_logger().info(
                f"{phase_name}: {tstep} steps, avg={avg*1000:.1f}ms/step"
            )

        return converged

    # ------------------------------------------------------------------
    # Voxel-based object attachment
    # ------------------------------------------------------------------

    def _attach_voxels_to_robot(self, current_state: JointState) -> bool:
        """
        Attach camera voxels near link6 as collision spheres on the robot.

        Algorithm (all GPU, no CPU round-trips):
          1. Compute link6 pose in world frame via MPC kinematics
          2. For each VoxelGrid in the world model, access xyzr_tensor and
             feature_tensor directly from VRAM
          3. Keep only occupied voxels (ESDF feature > threshold) that fall
             within grasp_attach_radius of the link6 origin
          4. Transform their world-frame positions into the link6 frame using
             Pose.inverse()
          5. Write them as [x, y, z, r] spheres into the "attached_object"
             link of the MPC kinematics config
             (mirrors KinematicsTensorConfig.attach_object used internally
             by MotionGen.attach_spheres_to_robot)

        The "attached_object" link must be pre-declared in the robot YAML
        with at least grasp_attach_max_spheres slots.
        """
        attach_radius  = self._param('grasp_attach_radius')
        max_spheres    = int(self._param('grasp_attach_max_spheres'))
        kc             = self.mpc.kinematics.kinematics_config

        # Verify the link exists
        if self.ATTACHED_LINK not in kc.link_name_to_idx_map:
            self.node.get_logger().warn(
                f"'{self.ATTACHED_LINK}' not found in robot kinematics config — "
                "add it to the robot YAML to enable object attachment"
            )
            return False

        # 1. Link6 pose in world frame
        kin_state  = self.mpc.compute_kinematics(current_state)
        link_poses = kin_state.link_poses
        if link_poses is None or self.GRASP_LINK not in link_poses:
            self.node.get_logger().error(
                f"'{self.GRASP_LINK}' not found in kinematics output"
            )
            return False

        link6_pose     = link_poses[self.GRASP_LINK]          # Pose, world frame
        link6_pos      = link6_pose.position.squeeze()        # [3], CUDA
        link6_inv_pose = link6_pose.inverse()                 # world→link6 transform

        # 2. Collect occupied voxels from all VoxelGrids in VRAM
        world_model = self.mpc.world_coll_checker.world_model
        voxel_positions = []  # list of [N, 3] CUDA tensors

        for vg in world_model.voxel:
            if vg.xyzr_tensor is None or vg.feature_tensor is None:
                continue

            xyz    = vg.xyzr_tensor[:, :3]           # [N, 3] — direct VRAM, no clone
            feats  = vg.feature_tensor                # [N]

            # Occupied: ESDF value above -0.5 * voxel_size
            occupied_mask = feats > (-0.5 * vg.voxel_size)

            # Within radius of link6
            dist_sq       = ((xyz - link6_pos) ** 2).sum(dim=1)
            radius_mask   = dist_sq < (attach_radius ** 2)

            mask = occupied_mask & radius_mask
            if mask.any():
                voxel_positions.append(xyz[mask])

        if not voxel_positions:
            self.node.get_logger().warn(
                f"No occupied voxels within {attach_radius:.3f}m of {self.GRASP_LINK}"
            )
            return False

        # 3. Merge and cap
        all_positions = torch.cat(voxel_positions, dim=0)   # [M, 3], CUDA
        if all_positions.shape[0] > max_spheres:
            # Uniform subsample to stay within pre-allocated slot count
            idx = torch.randperm(all_positions.shape[0], device=all_positions.device)
            all_positions = all_positions[idx[:max_spheres]]

        # 4. Transform world-frame positions into link6 frame
        #    Pose.batch_transform_points expects [B, N, 3]; we have [N, 3]
        n = all_positions.shape[0]
        pos_world_b = all_positions.unsqueeze(0)             # [1, N, 3]
        pos_link6_b = link6_inv_pose.batch_transform_points(pos_world_b)
        pos_link6   = pos_link6_b.squeeze(0)                 # [N, 3]

        # 5. Build sphere tensor [N, 4] = [x, y, z, r]
        # Use the voxel_size of the first grid as sphere radius
        sphere_r = world_model.voxel[0].voxel_size / 2.0
        radii    = torch.full((n, 1), sphere_r,
                              dtype=pos_link6.dtype, device=pos_link6.device)
        sphere_tensor = torch.cat([pos_link6, radii], dim=1)  # [N, 4]

        # Pad to max_spheres with inactive spheres (r=-100)
        if n < max_spheres:
            pad = torch.zeros(max_spheres - n, 4,
                              dtype=sphere_tensor.dtype, device=sphere_tensor.device)
            pad[:, 3] = -100.0
            sphere_tensor = torch.cat([sphere_tensor, pad], dim=0)

        # 6. Attach — mirrors KinematicsTensorConfig.attach_object()
        kc.attach_object(sphere_tensor=sphere_tensor, link_name=self.ATTACHED_LINK)

        self.node.get_logger().info(
            f"Attached {n} voxel spheres (r={sphere_r:.4f}m) to '{self.ATTACHED_LINK}'"
        )
        return True

    def _detach_from_robot(self):
        """Detach object spheres from robot (mirrors KinematicsTensorConfig.detach_object)."""
        try:
            kc = self.mpc.kinematics.kinematics_config
            if self.ATTACHED_LINK in kc.link_name_to_idx_map:
                kc.detach_object(self.ATTACHED_LINK)
                self.node.get_logger().info(f"Detached '{self.ATTACHED_LINK}'")
        except Exception as e:
            self.node.get_logger().error(f"Detach failed: {e}")

    # ------------------------------------------------------------------
    # Link6 collision toggle
    # ------------------------------------------------------------------

    def _toggle_link6(self, enable: bool):
        """Enable/disable link6 collision spheres on the MPC kinematics config."""
        try:
            kc = self.mpc.kinematics.kinematics_config
            if enable:
                kc.enable_link_spheres(self.GRASP_LINK)
            else:
                kc.disable_link_spheres(self.GRASP_LINK)
            self._link6_disabled = not enable
        except Exception as e:
            self.node.get_logger().error(
                f"{'enable' if enable else 'disable'} {self.GRASP_LINK} failed: {e}"
            )

    def _ensure_link6_enabled(self):
        """Re-enable link6 if left disabled (safety guard on error paths)."""
        if self._link6_disabled:
            self._toggle_link6(True)
            self.node.get_logger().info(f"{self.GRASP_LINK} collision re-enabled")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _offset_world_x(self, base_pose: Pose, offset_x: float) -> Pose:
        """Return base_pose translated by offset_x along world X axis (stays on CUDA)."""
        pos = base_pose.position
        offset = torch.tensor([offset_x, 0.0, 0.0], dtype=pos.dtype, device=pos.device)
        return Pose(position=pos + offset, quaternion=base_pose.quaternion)

    def _param(self, name: str):
        """Shorthand to read a float/int ROS parameter."""
        p = self.node.get_parameter(name).get_parameter_value()
        # Try float first, then integer
        v = p.double_value
        return v if v != 0.0 else p.integer_value

    def _close_gripper(self) -> bool:
        if self.gripper is not None:
            return self.gripper.close()
        time.sleep(self._param('grasp_gripper_close_time'))
        return True
