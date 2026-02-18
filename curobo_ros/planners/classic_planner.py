#!/usr/bin/env python3
"""
Classic trajectory planner using cuRobo MotionGen.

This planner generates a complete trajectory from start to goal in one shot,
then executes it in open-loop fashion.
"""

from typing import Optional

import torch
import numpy as np
from curobo.types.robot import JointState
from curobo.types.math import Pose
from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig, MotionGenResult

from .single_planner import SinglePlanner


class ClassicPlanner(SinglePlanner):
    """
    Classic motion generation planner.

    Uses cuRobo's MotionGen to generate a complete collision-free trajectory
    from start to goal, which is then executed in open-loop.

    This is the simplest MotionGen-based planner:
    - Takes a single target pose
    - Generates a full trajectory in one shot using plan_single()
    - Executes the trajectory as-is (no post-processing)

    Execution flow:
    1. plan() - Generate full trajectory using MotionGen.plan_single()
    2. execute() - Send trajectory to robot and monitor progress

    Example usage:
        >>> # After warmup
        >>> SinglePlanner.set_motion_gen(node.motion_gen)
        >>>
        >>> # Create planner
        >>> planner = ClassicPlanner(node, config_wrapper)
        >>>
        >>> # Plan
        >>> result = planner.plan(
        >>>     start_state=current_joint_state,
        >>>     goal_pose=target_pose,
        >>>     config={
        >>>         'max_attempts': 2,
        >>>         'timeout': 5.0,
        >>>         'time_dilation_factor': 0.5,
        >>>     }
        >>> )
        >>>
        >>> # Execute
        >>> if result.success:
        >>>     success = planner.execute(robot_context)
    """

    def get_planner_name(self) -> str:
        """Return planner name."""
        return "Classic Motion Generation"

    def _plan_trajectory(
        self,
        start_state: JointState,
        goal_request,
        config: dict
    ) -> MotionGenResult:
        """
        Generate trajectory using MotionGen.plan_single().

        This is the core planning logic for ClassicPlanner. It uses MotionGen's
        standard single-goal planning with collision avoidance.

        Args:
            start_state: Initial joint configuration
            goal_request: TrajectoryGeneration request
            config: Dictionary with keys:
                - max_attempts: Number of planning attempts (default: 1)
                - timeout: Planning timeout in seconds (default: 5.0)
                - time_dilation_factor: Trajectory time scaling (default: 0.5)

        Returns:
            MotionGenResult with trajectory and status
        """
        # Extract goal pose from request (ClassicPlanner uses target_pose)
        goal_pose = Pose.from_list([
            goal_request.target_pose.position.x,
            goal_request.target_pose.position.y,
            goal_request.target_pose.position.z,
            goal_request.target_pose.orientation.x,
            goal_request.target_pose.orientation.y,
            goal_request.target_pose.orientation.z,
            goal_request.target_pose.orientation.w
        ])

        # Extract config parameters
        max_attempts = config.get('max_attempts', 1)
        timeout = config.get('timeout', 5.0)
        time_dilation_factor = config.get('time_dilation_factor', 0.5)

        # Check for constraints
        pose_cost_metric = None
        if (hasattr(goal_request, 'trajectory_constraints') and
            goal_request.trajectory_constraints and
            len(goal_request.trajectory_constraints) == 6):

            # Check if at least one constraint is active
            if any(c == 1 for c in goal_request.trajectory_constraints):
                from curobo.rollout.cost.pose_cost import PoseCostMetric

                # Convert to PyTorch tensor (CuRobo requires torch.Tensor for .clone())
                hold_vec = torch.tensor(
                    goal_request.trajectory_constraints,
                    dtype=torch.float32
                )

                # Create PoseCostMetric with hold_vec_weight
                pose_cost_metric = PoseCostMetric(
                    hold_vec_weight=hold_vec,
                    hold_partial_pose=False  # Just constrain specified axes
                )

                self.node.get_logger().info(
                    f"ClassicPlanner: Constraints enabled {hold_vec.tolist()}"
                )

        self.node.get_logger().info(
            f"Planning with max_attempts={max_attempts}, "
            f"timeout={timeout}s, time_dilation={time_dilation_factor}"
        )

        # Plan trajectory using MotionGen
        result = self.motion_gen.plan_single(
            start_state,
            goal_pose,
            MotionGenPlanConfig(
                max_attempts=max_attempts,
                timeout=timeout,
                time_dilation_factor=time_dilation_factor,
                pose_cost_metric=pose_cost_metric,  # Pass constraint if defined
            ),
        )

        return result

    # Note: No need to override _process_trajectory() since we don't modify
    # the trajectory. The default implementation in SinglePlanner returns
    # the trajectory unchanged, which is exactly what we want.

    # Note: No need to override execute() since SinglePlanner provides
    # the complete open-loop execution logic that works for all children.

    # Note: No need to override cancel() since it's already implemented
    # in SinglePlanner and works for all children.
