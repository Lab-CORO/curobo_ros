#!/usr/bin/env python3
"""
Motion Retargeting — reactive teleop controller (cuRobo v2).

Thin wrapper over cuRobo's ``MotionRetargeter``. Like the MPC controller it is a
closed-loop sibling that follows a live stream of tool poses, but it solves with
**inverse kinematics** rather than a receding-horizon optimisation:

    solve_frame(goal):
        first frame  -> GLOBAL IK with num_seeds_global=64  (robust: finds a
                        solution on reachable poses where the MPC's hard-coded
                        single-seed retarget IK fails — see MPCController)
        next frames  -> LOCAL IK warm-started from the previous solution
                        (fast & smooth for a teleop pose stream)

This makes it the clean alternative to MPC for live tracking, free of the MPC
1-seed IK issue (see [[controller-interface-contract]]). All the ROS / robot /
perception control loop is inherited from :class:`ReactiveController`; this class
only implements the cuRobo-specific steps.

The solver is built from the node's single shared context (robot YAML + shared
Scene). ``solve_frame`` ignores the fed-back joint state (it warm-starts
internally), so unlike the MPC we don't need velocity/acceleration continuity.
"""

from typing import Any

from curobo.config_io import load_yaml
from curobo.types import JointState
from curobo._src.cost.tool_pose_criteria import ToolPoseCriteria
from curobo.motion_retargeter import MotionRetargeter, MotionRetargeterCfg

from .reactive_controller import ReactiveController


class RetargetController(ReactiveController):
    """Closed-loop teleop controller built on cuRobo ``MotionRetargeter`` (v2)."""

    def get_planner_name(self) -> str:
        return "Motion Retargeting (Teleop)"

    def get_config_parameters(self) -> list:
        return ['convergence_threshold', 'retarget_position_weight',
                'retarget_orientation_weight']

    # ---- cuRobo-specific hooks ------------------------------------------------

    def build_solver(self):
        cw = self.config_wrapper
        node = self.node
        w_pos = node.get_parameter('retarget_position_weight').get_parameter_value().double_value
        w_rot = node.get_parameter('retarget_orientation_weight').get_parameter_value().double_value
        use_mpc = node.get_parameter('retarget_use_mpc').get_parameter_value().bool_value

        tool = self._tool_frame()
        cfg = MotionRetargeterCfg.create(
            robot=cw.robot_config_file,
            tool_pose_criteria={
                tool: ToolPoseCriteria.track_position_and_orientation(
                    xyz=[w_pos, w_pos, w_pos], rpy=[w_rot, w_rot, w_rot],
                )
            },
            scene_model=cw.obstacle_manager.get_scene(),
            use_mpc=use_mpc,
            num_seeds_global=64,          # robust first-frame IK
            self_collision_check=True,
        )
        solver = MotionRetargeter(cfg)

        # Publish where the node expects it (symmetry with self.node.mpc) so the
        # world-update path can reach this solver.
        node.retargeter = solver
        node.get_logger().info(
            f"Retargeter built: tool={tool}, global IK 64 seeds, use_mpc={use_mpc}"
        )
        return solver

    def setup(self, start_state: JointState, goal_request: Any) -> bool:
        p = goal_request.target_pose
        raw = [
            p.position.x, p.position.y, p.position.z,
            p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z,
        ]
        # reset() clears the warm-start so the first solve_frame uses global IK.
        self.solver.reset()
        self.goal = self._set_target(raw)
        return True

    def step(self, current_state: JointState) -> JointState:
        # solve_frame tracks self.goal and warm-starts internally; it ignores the
        # fed-back current_state. on_target is the FK distance of the FRESH
        # solution (not the stale previous action).
        result = self.solver.solve_frame(self.goal)
        self._last_position_error = self._fk_position_error(result.joint_state)
        return result.joint_state

    def apply_live_goal(self, raw_goal) -> bool:
        # Replace the tracked goal; the next solve_frame warm-starts from the
        # current solution (local IK) for a smooth transition.
        self.goal = self._set_target(raw_goal)
        return True

    def update_world(self, scene) -> None:
        """Reload the shared Scene into the retargeter's IK solver(s)."""
        self.solver._global_ik_solver.update_world(scene)
        local_ik = getattr(self.solver, '_local_ik_solver', None)
        if local_ik is not None:
            local_ik.update_world(scene)

    # ---- helpers --------------------------------------------------------------

    def _tool_frame(self) -> str:
        """Read the tracked tool frame from the robot YAML (needed pre-build)."""
        cfg = load_yaml(self.config_wrapper.robot_config_file)
        kin = cfg.get('robot_cfg', cfg).get('kinematics', {})
        frames = kin.get('tool_frames')
        return frames[0] if frames else 'link6'
