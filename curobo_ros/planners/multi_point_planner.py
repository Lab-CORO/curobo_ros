#!/usr/bin/env python3
"""
Multi-point trajectory planner (v2: MotionPlanner.plan_pose).

v2 notes:
- PoseCostMetric is removed; whole-path axis constraints (`trajectory_constraints`)
  are re-wired via ToolPoseCriteria and applied to every segment. Per-waypoint
  `trajectories_contraints` remain unsupported (a single criteria spans the plan).
- MotionGenPlanConfig is gone; per-call params become kwargs on plan_pose().
- Kunz-Stilman retiming path is dropped (v2 exposes retiming differently and
  the v1 code was already falling back to raw stacked segments in practice).
  v2 users that need smoother blending can switch to a single goalset call.
"""

from typing import List

from curobo.types import JointState, Pose, GoalToolPose

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
        # Extract waypoints from request (MultiPointPlanner uses target_poses).
        # v2 Pose.from_list expects [x, y, z, qw, qx, qy, qz] (wxyz).
        tool_frame = self.motion_planner.tool_frames[0]
        waypoints: List[GoalToolPose] = []
        for pose_msg in goal_request.target_poses:
            p = Pose.from_list([
                pose_msg.position.x,
                pose_msg.position.y,
                pose_msg.position.z,
                pose_msg.orientation.w,
                pose_msg.orientation.x,
                pose_msg.orientation.y,
                pose_msg.orientation.z,
            ])
            waypoints.append(GoalToolPose.from_poses({tool_frame: p}))

        max_attempts = config.get('max_attempts', 1)
        connect_waypoints = config.get('connect_waypoints', False)

        # v2: a single ToolPoseCriteria applies to the whole plan, so per-waypoint
        # axis constraints can't be honored — only the whole-path
        # `trajectory_constraints` (applied to every segment below).
        if (hasattr(goal_request, 'trajectories_contraints')
                and goal_request.trajectories_contraints
                and any(c == 1 for c in goal_request.trajectories_contraints)):
            self.node.get_logger().warn(
                "MultiPointPlanner: per-waypoint `trajectories_contraints` are not "
                "supported in cuRobo v2 - use `trajectory_constraints` (whole path)."
            )

        # Hold the requested Cartesian axes along every segment; reset after
        # (the MotionPlanner is shared across planners).
        applied = self._apply_pose_constraints(goal_request)
        try:
            current_state = start_state.clone()
            combined_trajectory = None
            self._combined_trajectory = None
            last_result = None

            for i, goal in enumerate(waypoints):
                if not connect_waypoints:
                    current_state.velocity[:] = 0.0
                    current_state.acceleration[:] = 0.0

                result = self.motion_planner.plan_pose(
                    goal,
                    current_state.clone(),
                    max_attempts=max_attempts,
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

                # Build the next start state from the final waypoint of the segment.
                # segment.position is [B, T, D]; plan_pose requires a 2D [B, D]
                # current_state, so slice out the last timestep and reuse the
                # start_state's joint_names for the new JointState.
                last_pos = segment.position[..., -1, :]
                while last_pos.ndim > 2:
                    last_pos = last_pos[0]
                current_state = JointState.from_position(
                    last_pos.clone(),
                    joint_names=start_state.joint_names,
                )
                last_result = result

            self._combined_trajectory = combined_trajectory
            return last_result
        finally:
            if applied:
                self._reset_pose_criteria()

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
