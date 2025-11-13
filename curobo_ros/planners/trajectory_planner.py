#!/usr/bin/env python3
"""
Abstract base class for trajectory planning strategies.

This module defines the interface that all trajectory planners must implement.
Different planners can have different execution modes (open-loop vs closed-loop).
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, List, Any
from enum import Enum

import torch
from curobo.types.robot import JointState
from curobo.types.math import Pose


class ExecutionMode(Enum):
    """Defines how a trajectory is executed."""
    OPEN_LOOP = "open_loop"      # Generate full trajectory, then execute
    CLOSED_LOOP = "closed_loop"  # Iterative generation and execution (MPC-style)


@dataclass
class PlannerResult:
    """
    Result from trajectory planning.

    Attributes:
        success: Whether planning succeeded
        message: Human-readable status message
        trajectory: Full trajectory (for open-loop) or None (for closed-loop)
        metadata: Additional planner-specific data
    """
    success: bool
    message: str
    trajectory: Optional[Any] = None
    metadata: Optional[dict] = None


class TrajectoryPlanner(ABC):
    """
    Abstract base class for trajectory planning strategies.

    Each planner implements its own:
    - Planning algorithm (motion generation, MPC, etc.)
    - Execution mode (open-loop or closed-loop)
    - Interaction with robot context

    This design follows the Strategy Pattern, allowing different planning
    algorithms to be swapped dynamically.
    """

    def __init__(self, node, config_wrapper):
        """
        Initialize the planner.

        Args:
            node: ROS2 node for logging and parameters
            config_wrapper: Configuration wrapper with world/robot config
        """
        self.node = node
        self.config_wrapper = config_wrapper
        self._execution_mode = self._get_execution_mode()

    @abstractmethod
    def _get_execution_mode(self) -> ExecutionMode:
        """
        Define the execution mode for this planner.

        Returns:
            ExecutionMode indicating how trajectories are executed
        """
        pass

    @abstractmethod
    def plan(self, start_state: JointState, goal_pose: Pose, config: dict, robot_context: Optional[Any] = None) -> PlannerResult:
        """
        Generate a plan from start to goal.

        Args:
            start_state: Initial robot joint state
            goal_pose: Target end-effector pose
            config: Planner-specific configuration parameters
            robot_context: Optional RobotContext for trajectory visualization

        Returns:
            PlannerResult with success status and trajectory/metadata
        """
        pass

    @abstractmethod
    def execute(self, robot_context, goal_handle=None) -> bool:
        """
        Execute the planned trajectory.

        The execution behavior depends on the planner type:
        - Open-loop: Send full pre-computed trajectory
        - Closed-loop: Iterative planning and execution loop

        Args:
            robot_context: RobotContext for sending commands
            goal_handle: Optional ROS action goal handle for feedback

        Returns:
            True if execution completed successfully, False otherwise
        """
        pass

    def get_execution_mode(self) -> ExecutionMode:
        """
        Get the execution mode for this planner.

        Returns:
            ExecutionMode enum value
        """
        return self._execution_mode

    def is_open_loop(self) -> bool:
        """Check if this is an open-loop planner."""
        return self._execution_mode == ExecutionMode.OPEN_LOOP

    def is_closed_loop(self) -> bool:
        """Check if this is a closed-loop planner."""
        return self._execution_mode == ExecutionMode.CLOSED_LOOP

    @abstractmethod
    def get_planner_name(self) -> str:
        """
        Get a human-readable name for this planner.

        Returns:
            String name of the planner
        """
        pass

    def get_config_parameters(self) -> List[str]:
        """
        Get list of configuration parameters supported by this planner.

        Override this to provide parameter documentation.

        Returns:
            List of parameter names
        """
        return []
