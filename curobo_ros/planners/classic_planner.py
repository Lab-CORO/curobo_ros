#!/usr/bin/env python3
"""
Classic trajectory planner (v2: MotionPlanner.plan_pose).

v2 notes:
- MotionGen → MotionPlanner (wired via SinglePlanner._shared_motion_planner).
- MotionGenPlanConfig is gone: per-call params are kwargs on plan_pose().
- Pose → ToolPose, wrapped in GoalToolPose.
- PoseCostMetric was removed upstream in v2. Trajectory-axis constraints
  (`trajectory_constraints`) are re-wired via `ToolPoseCriteria` (held along
  the whole path) — see SinglePlanner._apply_pose_constraints.
"""

from curobo.types import JointState, Pose, GoalToolPose

from .single_planner import SinglePlanner


class ClassicPlanner(SinglePlanner):
    """
    Classic motion generation planner.

    Uses cuRobo's MotionPlanner to generate a complete collision-free trajectory
    from start to goal, which is then executed in open-loop.

    - Takes a single target pose.
    - Generates a full trajectory in one shot via plan_pose().
    - Executes the trajectory as-is (no post-processing).
    """

    def get_planner_name(self) -> str:
        return "Classic Motion Generation"

    def _plan_trajectory(
        self,
        start_state: JointState,
        goal_request,
        config: dict,
    ):
        """
        Generate trajectory using MotionPlanner.plan_pose().

        Args:
            start_state: Initial joint configuration.
            goal_request: TrajectoryGeneration request (uses target_pose).
            config: Dict with keys:
                - max_attempts (default 1)
                - timeout (default 5.0)
                - time_dilation_factor (default 0.5)

        Returns:
            MotionPlannerResult-like object.
        """
        # Extract goal pose from request. v2 Pose.from_list expects
        # [x, y, z, qw, qx, qy, qz] (quaternion wxyz).
        pose = Pose.from_list([
            goal_request.target_pose.position.x,
            goal_request.target_pose.position.y,
            goal_request.target_pose.position.z,
            goal_request.target_pose.orientation.w,
            goal_request.target_pose.orientation.x,
            goal_request.target_pose.orientation.y,
            goal_request.target_pose.orientation.z,
        ])
        tool_frame = self.motion_planner.tool_frames[0]
        goal = GoalToolPose.from_poses({tool_frame: pose})

        max_attempts = config.get('max_attempts', 1)

        self.node.get_logger().info(f"Planning with max_attempts={max_attempts}")

        # v2: Cartesian axis constraints via ToolPoseCriteria (held along the
        # whole path). Reset afterwards since the MotionPlanner is shared.
        applied = self._apply_pose_constraints(goal_request)
        try:
            return self.motion_planner.plan_pose(
                goal,
                start_state,
                max_attempts=max_attempts,
            )
        finally:
            if applied:
                self._reset_pose_criteria()

    # _process_trajectory(): default (no-op) from SinglePlanner is fine.
    # execute() / cancel(): inherited from SinglePlanner.
