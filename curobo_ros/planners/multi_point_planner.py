#!/usr/bin/env python3
"""
Multi-point trajectory planner (v2: MotionPlanner.plan_pose).

v2 notes:
- PoseCostMetric is removed; per-waypoint axis constraints are not wired.
  Incoming `trajectories_contraints` fields are ignored with a warning.
- MotionGenPlanConfig is gone; per-call params become kwargs on plan_pose().
- Kunz-Stilman retiming path is dropped (v2 exposes retiming differently and
  the v1 code was already falling back to raw stacked segments in practice).
  v2 users that need smoother blending can switch to a single goalset call.
"""

from typing import List, Optional

from curobo.types import JointState, ToolPose, GoalToolPose

from .single_planner import SinglePlanner


class MultiPointPlanner(SinglePlanner):
    """
    Multi-waypoint planner built on top of MotionPlanner.plan_pose().

    Plans each waypoint sequentially, then stacks interpolated segments into
    a single trajectory for open-loop execution.
    """

    def get_planner_name(self) -> str:
        return "Multi-Point Motion Generation"

    def _plan_trajectory(
        self,
        start_state: JointState,
        goal_request,
        config: dict,
    ):
        # Extract waypoints from request (MultiPointPlanner uses target_poses)
        waypoints: List[GoalToolPose] = []
        for pose_msg in goal_request.target_poses:
            tp = ToolPose.from_list([
                pose_msg.position.x,
                pose_msg.position.y,
                pose_msg.position.z,
                pose_msg.orientation.w,
                pose_msg.orientation.x,
                pose_msg.orientation.y,
                pose_msg.orientation.z,
            ])
            waypoints.append(GoalToolPose(tool_pose=tp))

        max_attempts = config.get('max_attempts', 1)
        timeout = config.get('timeout', 10.0)
        time_dilation_factor = config.get('time_dilation_factor', 0.5)
        connect_waypoints = config.get('connect_waypoints', False)

        if (hasattr(goal_request, 'trajectories_contraints')
                and goal_request.trajectories_contraints
                and any(c == 1 for c in goal_request.trajectories_contraints)):
            self.node.get_logger().warn(
                "MultiPointPlanner: trajectories_contraints requested but "
                "PoseCostMetric is not available in cuRobo v2 — ignoring."
            )

        current_state = start_state.clone()
        combined_trajectory = None
        self._combined_trajectory = None
        last_result = None

        for i, goal in enumerate(waypoints):
            if not connect_waypoints:
                current_state.velocity[:] = 0.0
                current_state.acceleration[:] = 0.0

            result = self.motion_planner.plan_pose(
                current_state.clone(),
                goal,
                max_attempts=max_attempts,
                timeout=timeout / max(1, len(waypoints)),
                time_dilation_factor=time_dilation_factor,
            )

            if not result.success.item():
                self.node.get_logger().error(
                    f"Failed to plan to waypoint {i}: {result.status}"
                )
                return result

            segment = (
                result.optimized_plan if connect_waypoints
                else result.get_interpolated_plan()
            )

            if combined_trajectory is None:
                combined_trajectory = segment
            else:
                combined_trajectory = combined_trajectory.stack(segment.clone())

            current_state = segment[-1].unsqueeze(0).clone()
            last_result = result

        self._combined_trajectory = combined_trajectory
        return last_result

    def _process_trajectory(self, trajectory: JointState, config: dict) -> JointState:
        """Return the stacked multi-waypoint trajectory built in _plan_trajectory."""
        if self._combined_trajectory is not None:
            self.node.get_logger().info(
                f"MultiPointPlanner: returning combined trajectory with "
                f"{len(self._combined_trajectory.position)} waypoints"
            )
            return self._combined_trajectory

        self.node.get_logger().warn("_combined_trajectory not set, using single segment")
        return trajectory

    def get_config_parameters(self) -> list:
        params = super().get_config_parameters()
        params.extend(['connect_waypoints'])
        return params
