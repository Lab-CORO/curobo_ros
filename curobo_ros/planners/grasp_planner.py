#!/usr/bin/env python3
"""
Native cuRobo v2 grasp pipeline (``MotionPlanner.plan_grasp``).

Wraps cuRobo's one-call grasp planner, which produces three legs from a goalset
of candidate grasp poses:

    goalset -> APPROACH (offset pose) -> GRASP (linear in) -> LIFT (linear out)

This is the native v2 replacement for the removed custom grasp. It is NOT a
SetPlanner strategy (it has its own ``plan_grasp`` action with grasp-specific
inputs); it reuses the node's single shared ``MotionPlanner`` (same instance as
the open-loop planners), the shared Scene and robot model — see
[[curobo-ros-thin-interface]].

Attach: after the grasp leg, the picked scene obstacle can be attached to the
robot's ``attached_object`` link as collision spheres (fitted by ``sphere_fit``
via cuRobo's attachment_manager). The object then moves with the arm for
subsequent open-loop transport planning, until ``detach`` releases it. The robot
YAML must declare the ``attached_object`` link (see m1013.yml).
"""

import time
import traceback

import torch
from curobo.types import JointState, GoalToolPose

from curobo_msgs.action import GraspPlan

ATTACH_LINK = "attached_object"


class GraspPlanner:
    """Plans + executes a native cuRobo grasp and (optionally) attaches the object."""

    def __init__(self, node, config_wrapper):
        self.node = node
        self.config_wrapper = config_wrapper
        self._attached_name = None  # name of the currently attached obstacle

    @property
    def motion_planner(self):
        """The node's shared, warmed-up MotionPlanner (guaranteed by the node)."""
        return self.node.motion_planner

    def _attachment_manager(self):
        """cuRobo AttachmentManager for the shared MotionPlanner.

        In this build ``MotionPlanner.attachment_manager`` forwards to
        ``trajopt_solver.attachment_manager``, but TrajOptSolver composes
        SolverCore and doesn't re-expose it — reach into ``.core`` instead.
        """
        mp = self.motion_planner
        am = getattr(mp, 'attachment_manager', None)  # property may raise -> None
        if am is None:
            am = mp.trajopt_solver.core.attachment_manager
        return am

    # ------------------------------------------------------------------
    # Planning
    # ------------------------------------------------------------------

    def plan(self, start_state: JointState, goal_request):
        """Run ``plan_grasp`` and return the cuRobo ``GraspPlanResult``."""
        mp = self.motion_planner
        grasp_poses = self._build_goalset(goal_request.grasp_candidates, mp.tool_frames)
        return mp.plan_grasp(
            grasp_poses=grasp_poses,
            current_state=start_state,
            grasp_approach_axis=goal_request.approach_axis or 'z',
            grasp_approach_offset=goal_request.approach_offset,
            grasp_approach_in_tool_frame=goal_request.approach_in_tool_frame,
            grasp_lift_axis=goal_request.lift_axis or 'z',
            grasp_lift_offset=goal_request.lift_offset,
            grasp_lift_in_tool_frame=goal_request.lift_in_tool_frame,
            plan_approach_to_grasp=goal_request.plan_approach_to_grasp,
            plan_grasp_to_lift=goal_request.plan_grasp_to_lift,
        )

    def _build_goalset(self, poses, tool_frames) -> GoalToolPose:
        """List[geometry_msgs/Pose] -> GoalToolPose of shape [B=1, H=1, L=1, G, 3/4]."""
        device = self.config_wrapper._device
        dtype = self.config_wrapper._ops_dtype
        n = len(poses)
        if n == 0:
            raise ValueError("grasp_candidates is empty (need >=1 candidate grasp pose)")
        position = torch.zeros((1, n, 3), device=device, dtype=dtype)
        quaternion = torch.zeros((1, n, 4), device=device, dtype=dtype)
        for i, p in enumerate(poses):
            position[0, i, :] = torch.tensor(
                [p.position.x, p.position.y, p.position.z], device=device, dtype=dtype)
            quaternion[0, i, :] = torch.tensor(
                [p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z],
                device=device, dtype=dtype)
        return GoalToolPose(
            tool_frames=list(tool_frames),
            position=position.unsqueeze(1).unsqueeze(1),    # [1,1,1,G,3]
            quaternion=quaternion.unsqueeze(1).unsqueeze(1),
        )

    # ------------------------------------------------------------------
    # Attach / detach (collision spheres on the attached_object link)
    # ------------------------------------------------------------------

    def attach(self, object_name: str, grasp_end_state: JointState) -> None:
        """Fit + attach the named scene obstacle to the arm at the grasp config.

        The obstacle is fetched from the ObstacleManager's authoritative Scene
        (so it works even if the solver's loaded scene_model is out of sync) and
        passed directly to ``attach`` (not ``attach_from_scene``, which looks up
        by name in the solver's scene_model). ``disable_obstacle_names`` also
        disables the world obstacle so it isn't double-counted with the attached
        spheres. Affects the shared MotionPlanner only (open-loop transport);
        MPC/retarget keep their own kinematics and do not carry the object.
        """
        scene = self.config_wrapper.obstacle_manager.get_scene()
        obstacle = self._find_obstacle(scene, object_name)
        if obstacle is None:
            raise ValueError(f"Obstacle '{object_name}' not found in the scene")
        self._attachment_manager().attach(
            joint_states=grasp_end_state,
            obstacles=[obstacle],
            link_name=ATTACH_LINK,
            disable_obstacle_names=[object_name],
        )
        self._attached_name = object_name
        self.node.get_logger().info(
            f"Grasp: attached '{object_name}' to link '{ATTACH_LINK}'")

    @staticmethod
    def _find_obstacle(scene, name: str):
        """Find an obstacle by name across the Scene's typed buckets.

        The ObstacleManager stores obstacles in typed lists (cuboid/sphere/…),
        not in Scene.objects, so Scene.get_obstacle (which reads .objects) misses
        runtime-added ones — search the buckets directly.
        """
        for bucket in ('cuboid', 'sphere', 'capsule', 'cylinder', 'mesh'):
            for obj in (getattr(scene, bucket, None) or []):
                if obj.name == name:
                    return obj
        if getattr(scene, 'objects', None):
            return scene.get_obstacle(name)
        return None

    def detach(self) -> str:
        """Release the attached object (reset link spheres, re-enable the obstacle)."""
        self._attachment_manager().detach(ATTACH_LINK)
        released = self._attached_name
        self._attached_name = None
        self.node.get_logger().info(f"Grasp: detached '{released}' from '{ATTACH_LINK}'")
        return released or ""

    # ------------------------------------------------------------------
    # Execution (stream approach -> grasp -> [attach] -> lift)
    # ------------------------------------------------------------------

    def execute(self, robot_context, result, goal_request, goal_handle=None) -> bool:
        """Stream the planned legs to the robot; attach the object after the grasp.

        cuRobo interpolated trajectories are padded to the interpolation buffer;
        each leg is trimmed to its ``*_interpolated_last_tstep`` so the emulator
        does not replay the repeated tail.
        """
        try:
            legs = []
            if result.approach_interpolated_trajectory is not None:
                legs.append(("APPROACH", result.approach_interpolated_trajectory,
                             result.approach_interpolated_last_tstep))
            if result.grasp_interpolated_trajectory is not None:
                legs.append(("GRASP", result.grasp_interpolated_trajectory,
                             result.grasp_interpolated_last_tstep))

            n_legs = len(legs) + (1 if result.lift_interpolated_trajectory is not None else 0)
            done = 0

            for phase, js, last in legs:
                if not self._stream(robot_context, js, last, goal_handle, phase, done, n_legs):
                    return False
                done += 1
                # Attach right after the grasp leg, at its final configuration.
                if phase == "GRASP" and goal_request.attach_object_name:
                    self._publish(goal_handle, "ATTACH", done / max(n_legs, 1))
                    self.attach(goal_request.attach_object_name,
                                self._last_state(js, last))

            if result.lift_interpolated_trajectory is not None:
                if not self._stream(robot_context, result.lift_interpolated_trajectory,
                                    result.lift_interpolated_last_tstep,
                                    goal_handle, "LIFT", done, n_legs):
                    return False

            self._publish(goal_handle, "DONE", 1.0)
            return True
        except Exception as e:
            self.node.get_logger().error(f"Grasp execution error: {e}")
            self.node.get_logger().error(traceback.format_exc())
            robot_context.stop_robot()
            return False

    def _stream(self, robot_context, js, last_tstep, goal_handle, phase, leg_index, n_legs) -> bool:
        pos_list = self._to_2d_list(js.position)
        vel_list = self._to_2d_list(js.velocity) or [[0.0] * len(pos_list[0]) for _ in pos_list]
        acc_list = self._to_2d_list(js.acceleration) or [[0.0] * len(pos_list[0]) for _ in pos_list]

        n = self._last_index(last_tstep, len(pos_list))
        pos_list, vel_list, acc_list = pos_list[:n + 1], vel_list[:n + 1], acc_list[:n + 1]
        joint_names = js.joint_names or robot_context.get_joint_name()

        robot_context.set_command(joint_names, vel_list, acc_list, pos_list)
        robot_context.send_trajectrory()

        while robot_context.get_progression() < 1.0:
            if goal_handle is not None and not goal_handle.is_active:
                robot_context.stop_robot()
                return False
            leg_prog = robot_context.get_progression()
            self._publish(goal_handle, phase, (leg_index + leg_prog) / max(n_legs, 1))
            time.sleep(0.01)
        time.sleep(0.05)  # let the emulator settle on the final waypoint
        return True

    @staticmethod
    def _last_index(last_tstep, n_points: int) -> int:
        """Real final index of an interpolated leg (handles padding / None)."""
        if last_tstep is None:
            return n_points - 1
        try:
            idx = int(last_tstep.reshape(-1)[0].item() if hasattr(last_tstep, 'reshape')
                      else last_tstep)
        except Exception:
            return n_points - 1
        return max(0, min(idx, n_points - 1))

    def _publish(self, goal_handle, phase, progression) -> None:
        if goal_handle is None:
            return
        fb = GraspPlan.Feedback()
        fb.phase = phase
        fb.progression = float(min(max(progression, 0.0), 1.0))
        goal_handle.publish_feedback(fb)

    # ------------------------------------------------------------------
    # Tensor / message helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _to_2d_list(t):
        """Flatten a [.., T, D] tensor to list[list[float]] (or None)."""
        if t is None:
            return None
        while t.ndim > 2:
            t = t[0]
        return t.detach().cpu().tolist()

    @classmethod
    def _last_state(cls, js: JointState, last_tstep=None) -> JointState:
        """JointState at the real final waypoint of an interpolated leg ([1, dof])."""
        pos = js.position
        while pos.ndim > 2:
            pos = pos[0]
        n = cls._last_index(last_tstep, pos.shape[0])
        return JointState.from_position(pos[n].unsqueeze(0))

    @classmethod
    def traj_to_msgs(cls, js, last_tstep=None):
        """Convert an interpolated JointState trajectory to sensor_msgs/JointState[].

        Trims the cuRobo interpolation padding when ``last_tstep`` is given.
        """
        from sensor_msgs.msg import JointState as JointStateMsg
        if js is None:
            return []
        pos = cls._to_2d_list(js.position)
        vel = cls._to_2d_list(js.velocity)
        n = cls._last_index(last_tstep, len(pos))
        pos = pos[:n + 1]
        if vel is not None:
            vel = vel[:n + 1]
        names = list(js.joint_names) if getattr(js, 'joint_names', None) else []
        msgs = []
        for i in range(len(pos)):
            m = JointStateMsg()
            m.name = names
            m.position = pos[i]
            if vel is not None:
                m.velocity = vel[i]
            msgs.append(m)
        return msgs

    @staticmethod
    def dt_of(result) -> float:
        """A representative interpolation dt from the grasp result (float)."""
        for attr in ('approach_trajectory_dt', 'grasp_trajectory_dt', 'lift_trajectory_dt'):
            dt = getattr(result, attr, None)
            if dt is not None:
                return float(dt.item() if hasattr(dt, 'item') else dt)
        return 0.0
