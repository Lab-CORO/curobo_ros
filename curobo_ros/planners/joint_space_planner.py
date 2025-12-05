#!/usr/bin/env python3
"""
Joint space trajectory planner using cuRobo MotionGen.

This planner generates collision-free trajectories between joint configurations,
using MotionGen's plan_single_js() for pure joint-space planning without IK.
"""

from typing import Optional

import torch
from curobo.types.robot import JointState
from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig, MotionGenResult

from .single_planner import SinglePlanner


class JointSpacePlanner(SinglePlanner):
    """
    Joint space motion generation planner.

    Uses cuRobo's MotionGen.plan_single_js() to generate collision-free
    trajectories directly in joint space, without requiring IK solving.

    This planner is ideal for:
    - Moving between known joint configurations
    - Tasks where end-effector pose is flexible
    - Avoiding IK singularities or multiple IK solutions
    - Fast planning when goal is already in joint space

    Execution flow:
    1. plan() - Generate trajectory using MotionGen.plan_single_js()
    2. execute() - Send trajectory to robot and monitor progress

    Example usage:
        >>> # After warmup
        >>> SinglePlanner.set_motion_gen(node.motion_gen)
        >>>
        >>> # Create planner
        >>> planner = JointSpacePlanner(node, config_wrapper)
        >>>
        >>> # Plan
        >>> from curobo_msgs.srv import TrajectoryGeneration
        >>> request = TrajectoryGeneration.Request()
        >>> request.target_joint_positions = [0.0, -0.5, 0.0, -1.5, 0.0, 1.0, 0.0]
        >>>
        >>> result = planner.plan(
        >>>     start_state=current_joint_state,
        >>>     goal_request=request,
        >>>     config={
        >>>         'max_attempts': 2,
        >>>         'timeout': 5.0,
        >>>         'time_dilation_factor': 0.5,
        >>>         'enable_graph': True,  # Use graph planner for collision avoidance
        >>>         'enable_opt': True,    # Enable trajectory optimization
        >>>     }
        >>> )
        >>>
        >>> # Execute
        >>> if result.success:
        >>>     success = planner.execute(robot_context)
    """

    def get_planner_name(self) -> str:
        """Return planner name."""
        return "Joint Space Motion Generation"

    def _plan_trajectory(
        self,
        start_state: JointState,
        goal_request,
        config: dict
    ) -> MotionGenResult:
        """
        Generate trajectory using MotionGen.plan_single_js().

        This is the core planning logic for JointSpacePlanner. It plans
        directly in joint space without IK, avoiding singularities.

        Args:
            start_state: Initial joint configuration
            goal_request: TrajectoryGeneration request (uses target_joint_positions)
            config: Dictionary with keys:
                - max_attempts: Number of planning attempts (default: 1)
                - timeout: Planning timeout in seconds (default: 5.0)
                - time_dilation_factor: Trajectory time scaling (default: 0.5)
                - enable_graph: Use graph planner for collision avoidance (default: True)
                - enable_opt: Enable trajectory optimization (default: True)

        Returns:
            MotionGenResult with trajectory and status

        Raises:
            ValueError: If target_joint_positions is missing, empty, wrong size, or invalid
        """
        # Validate goal request has target_joint_positions
        if not hasattr(goal_request, 'target_joint_positions'):
            raise ValueError(
                "JointSpacePlanner requires 'target_joint_positions' field in request. "
                "Ensure you're using an updated TrajectoryGeneration.srv with this field."
            )

        goal_joint_positions = goal_request.target_joint_positions

        # Validate non-empty
        if not goal_joint_positions or len(goal_joint_positions) == 0:
            raise ValueError(
                "target_joint_positions is empty. Provide target joint configuration."
            )

        # Validate joint count matches robot DOF
        robot_dof = self.motion_gen.kinematics.get_dof()
        if len(goal_joint_positions) != robot_dof:
            raise ValueError(
                f"Joint count mismatch: received {len(goal_joint_positions)} joints, "
                f"but robot has {robot_dof} DOF"
            )

        # Validate no NaN/Inf values
        if any(not (-1e6 < x < 1e6) or x != x for x in goal_joint_positions):
            raise ValueError(
                f"Invalid joint positions detected (NaN/Inf): {goal_joint_positions}"
            )

        # Convert to JointState
        # Follow pattern from unified_planner_node.py: wrap in [] for batch dimension
        goal_state = JointState.from_position(
            torch.Tensor([goal_joint_positions])
            .to(device=self.motion_gen.tensor_args.device)
        )

        # Extract config parameters
        max_attempts = config.get('max_attempts', 1)
        timeout = config.get('timeout', 5.0)
        time_dilation_factor = config.get('time_dilation_factor', 0.5)
        enable_graph = config.get('enable_graph', True)  # Enable graph planner for collision avoidance
        enable_opt = config.get('enable_opt', True)  # Enable trajectory optimization

        # Log planning parameters for debugging
        start_pos = start_state.position[0].cpu().tolist()
        goal_pos = list(goal_joint_positions)

        self.node.get_logger().info(
            f"Planning joint space trajectory:"
        )
        self.node.get_logger().info(
            f"  Start: {[f'{x:.3f}' for x in start_pos]}"
        )
        self.node.get_logger().info(
            f"  Goal:  {[f'{x:.3f}' for x in goal_pos]}"
        )
        self.node.get_logger().info(
            f"  Config: max_attempts={max_attempts}, timeout={timeout}s, "
            f"time_dilation={time_dilation_factor}, enable_graph={enable_graph}, enable_opt={enable_opt}"
        )

        # Plan trajectory using MotionGen.plan_single_js()
        # Signature: plan_single_js(start_state, goal_state, plan_config)
        # enable_graph=True helps find collision-free paths through obstacles
        # The graph planner generates a seed trajectory, then trajopt refines it
        result = self.motion_gen.plan_single_js(
            start_state,
            goal_state,
            MotionGenPlanConfig(
                max_attempts=max_attempts,
                timeout=timeout,
                time_dilation_factor=time_dilation_factor,
                enable_graph=enable_graph,  # Use graph planner for collision avoidance
                enable_opt=enable_opt,  # Enable trajectory optimization
            ),
        )

        return result

    # Note: No need to override _process_trajectory() since we don't modify
    # the trajectory. The default implementation in SinglePlanner returns
    # the trajectory unchanged, which is exactly what we want.

    # Note: No need to override execute() since SinglePlanner provides
    # the complete open-loop execution logic that works for all children.

    # Note: get_config_parameters() is inherited from SinglePlanner and
    # includes all standard parameters (max_attempts, timeout, etc.)
