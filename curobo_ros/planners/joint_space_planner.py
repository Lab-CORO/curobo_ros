#!/usr/bin/env python3
"""
Joint space trajectory planner (v2).

v2 notes:
- MotionGen.plan_single_js() → MotionPlanner.plan_cspace() (joint goal).
- MotionGenPlanConfig is gone; per-call params are kwargs on plan_cspace().
- v2 plan_cspace() no longer accepts timeout/time_dilation_factor/
  enable_graph/enable_opt — those tunables live on the trajopt YAML.
"""

from typing import Optional

import torch
from curobo.types import JointState

from .single_planner import SinglePlanner


class JointSpacePlanner(SinglePlanner):
    """
    Joint-space planner using MotionPlanner.plan_joint_state() (v2).

    Plans directly in joint space — no IK, no singularities to worry about.
    Ideal when the goal is already expressed as a joint configuration.
    """

    def get_planner_name(self) -> str:
        return "Joint Space Motion Generation"

    def _plan_trajectory(
        self,
        start_state: JointState,
        goal_request,
        config: dict,
    ):
        if not hasattr(goal_request, 'target_joint_positions'):
            raise ValueError(
                "JointSpacePlanner requires 'target_joint_positions' in the request."
            )

        goal_joint_positions = goal_request.target_joint_positions
        if not goal_joint_positions:
            raise ValueError("target_joint_positions is empty.")

        robot_dof = self.motion_planner.kinematics.get_dof()
        if len(goal_joint_positions) != robot_dof:
            raise ValueError(
                f"Joint count mismatch: received {len(goal_joint_positions)} joints, "
                f"but robot has {robot_dof} DOF"
            )
        if any(not (-1e6 < x < 1e6) or x != x for x in goal_joint_positions):
            raise ValueError(
                f"Invalid joint positions (NaN/Inf): {goal_joint_positions}"
            )

        goal_state = JointState.from_position(
            torch.tensor(
                [goal_joint_positions],
                dtype=start_state.position.dtype,
                device=start_state.position.device,
            )
        )

        max_attempts = config.get('max_attempts', 1)
        enable_graph_attempt = config.get('enable_graph_attempt', 1)

        start_pos = start_state.position[0].cpu().tolist()
        goal_pos = list(goal_joint_positions)
        self.node.get_logger().info("Planning joint space trajectory:")
        self.node.get_logger().info(f"  Start: {[f'{x:.3f}' for x in start_pos]}")
        self.node.get_logger().info(f"  Goal:  {[f'{x:.3f}' for x in goal_pos]}")
        self.node.get_logger().info(
            f"  Config: max_attempts={max_attempts}, "
            f"enable_graph_attempt={enable_graph_attempt}"
        )

        return self.motion_planner.plan_cspace(
            goal_state,
            start_state,
            max_attempts=max_attempts,
            enable_graph_attempt=enable_graph_attempt,
        )
