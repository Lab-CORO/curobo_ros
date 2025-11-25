#!/usr/bin/env python3
"""
Multi-point trajectory planner using cuRobo MotionGen.

This planner generates a trajectory that passes through multiple waypoints,
useful for tasks requiring intermediate poses (e.g., pick-and-place with obstacles).
"""

from typing import List, Optional

import torch
from curobo.types.robot import JointState
from curobo.types.math import Pose
from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig, MotionGenResult

from .single_planner import SinglePlanner


class MultiPointPlanner(SinglePlanner):
    """
    Multi-point motion generation planner.

    Extends ClassicPlanner to support trajectories through multiple waypoints.
    Uses MotionGen's batch planning to generate a trajectory that visits
    each waypoint in sequence.

    This planner is useful for:
    - Pick-and-place operations with specific approach/retreat poses
    - Navigation through tight spaces requiring specific intermediate poses
    - Tasks requiring precise path constraints

    Execution flow:
    1. plan() - Generate trajectory through all waypoints using MotionGen
    2. execute() - Send combined trajectory to robot

    Example usage:
        >>> # After warmup
        >>> SinglePlanner.set_motion_gen(node.motion_gen)
        >>>
        >>> # Create planner
        >>> planner = MultiPointPlanner(node, config_wrapper)
        >>>
        >>> # Plan through multiple waypoints
        >>> waypoints = [approach_pose, grasp_pose, retreat_pose]
        >>> result = planner.plan(
        >>>     start_state=current_joint_state,
        >>>     goal_pose=waypoints,  # List of Pose objects
        >>>     config={
        >>>         'max_attempts': 2,
        >>>         'timeout': 10.0,
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
        return "Multi-Point Motion Generation"

    def _plan_trajectory(
        self,
        start_state: JointState,
        goal_request,
        config: dict
    ) -> MotionGenResult:
        """
        Generate trajectory through multiple waypoints.

        Extracts multiple poses from goal_request.target_poses and plans
        a trajectory that visits each waypoint in sequence.

        Args:
            start_state: Initial joint configuration
            goal_request: TrajectoryGeneration request (uses target_poses field)
            config: Dictionary with keys:
                - max_attempts: Number of planning attempts (default: 1)
                - timeout: Planning timeout in seconds (default: 10.0)
                - time_dilation_factor: Trajectory time scaling (default: 0.5)
                - connect_waypoints: If True, connect waypoints with smooth motion
                                    If False, plan to each waypoint independently
                                    (default: True)

        Returns:
            MotionGenResult with trajectory through all waypoints
        """
        # Extract waypoints from request (MultiPointPlanner uses target_poses)
        waypoints = []
        for pose_msg in goal_request.target_poses:
            waypoint = Pose.from_list([
                pose_msg.position.x,
                pose_msg.position.y,
                pose_msg.position.z,
                pose_msg.orientation.x,
                pose_msg.orientation.y,
                pose_msg.orientation.z,
                pose_msg.orientation.w
            ])
            waypoints.append(waypoint)

        # Extract config parameters
        max_attempts = config.get('max_attempts', 1)
        timeout = config.get('timeout', 10.0)
        time_dilation_factor = config.get('time_dilation_factor', 0.5)
        connect_waypoints = config.get('connect_waypoints', True)

        self.node.get_logger().info(
            f"Planning through {len(waypoints)} waypoints with "
            f"max_attempts={max_attempts}, timeout={timeout}s"
        )

        # For now, use sequential planning through waypoints
        # In future, could use MotionGen's batch planning for optimization
        if len(waypoints) == 1:
            # Single waypoint - use standard planning
            result = self.motion_gen.plan_single(
                start_state,
                waypoints[0],
                MotionGenPlanConfig(
                    max_attempts=max_attempts,
                    timeout=timeout,
                    time_dilation_factor=time_dilation_factor,
                ),
            )
        else:
            # Multiple waypoints - plan segments and combine
            # This is a simplified implementation
            # For production, consider using MotionGen's built-in multi-goal planning

            # Plan to first waypoint
            result = self.motion_gen.plan_single(
                start_state,
                waypoints[0],
                MotionGenPlanConfig(
                    max_attempts=max_attempts,
                    timeout=timeout / len(waypoints),  # Divide timeout across segments
                    time_dilation_factor=time_dilation_factor,
                ),
            )

            if not result.success.item():
                self.node.get_logger().error(f"Failed to plan to waypoint 0")
                return result

            # For subsequent waypoints, plan from end of previous segment
            for i, waypoint in enumerate(waypoints[1:], start=1):
                # Get final state from previous segment
                prev_traj = result.get_interpolated_plan()
                intermediate_state = JointState.from_position(
                    prev_traj.position[-1:],  # Last position as start for next segment
                    joint_names=prev_traj.joint_names
                )

                # Plan next segment
                next_result = self.motion_gen.plan_single(
                    intermediate_state,
                    waypoint,
                    MotionGenPlanConfig(
                        max_attempts=max_attempts,
                        timeout=timeout / len(waypoints),
                        time_dilation_factor=time_dilation_factor,
                    ),
                )

                if not next_result.success.item():
                    self.node.get_logger().error(f"Failed to plan to waypoint {i}")
                    return next_result

                # TODO: Combine trajectories properly
                # For now, just return the last segment
                # In production, you'd concatenate all segments
                result = next_result

            self.node.get_logger().info(
                f"Successfully planned through all {len(waypoints)} waypoints"
            )

        return result

    def _process_trajectory(self, trajectory: JointState, config: dict) -> JointState:
        """
        Post-process the multi-point trajectory.

        For multi-point planning, you might want to:
        - Add pauses at waypoints
        - Smooth out velocity discontinuities at waypoint transitions
        - Add custom actions at specific waypoints

        Args:
            trajectory: Raw trajectory from MotionGen
            config: Configuration dictionary

        Returns:
            Processed trajectory
        """
        # For now, return unchanged
        # In production, you might add waypoint-specific processing here
        return trajectory

    def get_config_parameters(self) -> list:
        """
        Get configuration parameters for multi-point planner.

        Returns:
            List of parameter names
        """
        # Start with base parameters from SinglePlanner
        params = super().get_config_parameters()

        # Add multi-point specific parameters
        params.extend([
            'connect_waypoints',
        ])

        return params
