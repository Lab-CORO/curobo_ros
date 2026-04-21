#!/usr/bin/env python3
"""
Classic trajectory planner (v2: MotionPlanner.plan_pose).

v2 notes:
- MotionGen → MotionPlanner (wired via SinglePlanner._shared_motion_planner).
- MotionGenPlanConfig is gone: per-call params are kwargs on plan_pose().
- Pose → ToolPose, wrapped in GoalToolPose.
- PoseCostMetric was removed upstream in v2. Trajectory-axis constraints
  are not yet re-wired; the old `trajectory_constraints` field is now a
  no-op with a warning. Revisit once the v2 constraint API stabilises.
"""

from typing import Optional

from curobo.types import JointState, ToolPose, GoalToolPose

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
        # Extract goal pose from request (ClassicPlanner uses target_pose).
        # v2 ToolPose expects [x, y, z, qw, qx, qy, qz].
        tool_pose = ToolPose.from_list([
            goal_request.target_pose.position.x,
            goal_request.target_pose.position.y,
            goal_request.target_pose.position.z,
            goal_request.target_pose.orientation.w,
            goal_request.target_pose.orientation.x,
            goal_request.target_pose.orientation.y,
            goal_request.target_pose.orientation.z,
        ])
        goal = GoalToolPose(tool_pose=tool_pose)

        max_attempts = config.get('max_attempts', 1)
        timeout = config.get('timeout', 5.0)
        time_dilation_factor = config.get('time_dilation_factor', 0.5)

        # v2: PoseCostMetric is removed. Warn if the client still populates
        # trajectory_constraints so the behavior change is visible.
        if (hasattr(goal_request, 'trajectory_constraints')
                and goal_request.trajectory_constraints
                and any(c == 1 for c in goal_request.trajectory_constraints)):
            self.node.get_logger().warn(
                "ClassicPlanner: trajectory_constraints requested but "
                "PoseCostMetric is not available in cuRobo v2 — ignoring."
            )

        self.node.get_logger().info(
            f"Planning with max_attempts={max_attempts}, "
            f"timeout={timeout}s, time_dilation={time_dilation_factor}"
        )

        return self.motion_planner.plan_pose(
            start_state,
            goal,
            max_attempts=max_attempts,
            timeout=timeout,
            time_dilation_factor=time_dilation_factor,
        )

    # _process_trajectory(): default (no-op) from SinglePlanner is fine.
    # execute() / cancel(): inherited from SinglePlanner.
