#!/usr/bin/env python3
"""
Abstract interface for gripper control.

This module defines the standard interface that all gripper implementations
must follow. Different robot grippers can implement this interface to provide
consistent gripper control across different platforms.
"""

from abc import ABC, abstractmethod


class GripperInterface(ABC):
    """
    Abstract base class for gripper control.

    This interface provides a unified way to control different gripper types
    (parallel jaw, vacuum, multi-finger, etc.) across different robot platforms.

    Implementations should handle:
    - Command sending (ROS topics/services, direct control, etc.)
    - State feedback (position, force, etc.)
    - Timing and synchronization

    Example usage:
        >>> gripper = DoosanGripper(node)
        >>> gripper.open()  # Opens gripper
        >>> success = gripper.close()  # Closes gripper
        >>> pos = gripper.get_state()  # Get current position (0.0-1.0)
    """

    @abstractmethod
    def open(self) -> bool:
        """
        Open the gripper to maximum width.

        This method should:
        1. Send command to open gripper
        2. Wait for gripper to reach open position
        3. Return success/failure status

        Returns:
            True if gripper opened successfully, False otherwise

        Raises:
            Can raise exceptions for communication errors
        """
        pass

    @abstractmethod
    def close(self) -> bool:
        """
        Close the gripper.

        This method should:
        1. Send command to close gripper
        2. Wait for gripper to reach closed position or detect object
        3. Return success/failure status

        Returns:
            True if gripper closed successfully, False otherwise

        Note:
            For force-controlled grippers, "close" may stop when an object
            is detected rather than fully closing.

        Raises:
            Can raise exceptions for communication errors
        """
        pass

    @abstractmethod
    def get_state(self) -> float:
        """
        Get current gripper position.

        Returns:
            Gripper position as a float:
            - 0.0 = fully closed
            - 1.0 = fully open
            - Values in between represent partial opening

        Note:
            The exact mapping depends on gripper type:
            - Parallel jaw: 0.0 = jaws touching, 1.0 = max width
            - Vacuum: 0.0 = vacuum on, 1.0 = vacuum off
            - Multi-finger: 0.0 = fingers closed, 1.0 = fingers open
        """
        pass

    def set_position(self, position: float) -> bool:
        """
        Set gripper to a specific position (optional).

        Not all grippers support arbitrary position control.
        Default implementation only supports binary open/close.

        Args:
            position: Target position (0.0 = closed, 1.0 = open)

        Returns:
            True if successful

        Raises:
            NotImplementedError if gripper doesn't support position control
        """
        if position >= 0.5:
            return self.open()
        else:
            return self.close()

    def get_force(self) -> float:
        """
        Get current gripping force (optional).

        Not all grippers have force sensors.
        Default implementation returns 0.0.

        Returns:
            Gripping force in Newtons, or 0.0 if not supported
        """
        return 0.0

    def is_open(self) -> bool:
        """
        Check if gripper is fully open.

        Returns:
            True if gripper is open (state >= 0.9)
        """
        return self.get_state() >= 0.9

    def is_closed(self) -> bool:
        """
        Check if gripper is fully closed.

        Returns:
            True if gripper is closed (state <= 0.1)
        """
        return self.get_state() <= 0.1

    def is_grasping(self) -> bool:
        """
        Check if gripper is currently grasping an object (optional).

        Default implementation uses force threshold.
        Override for grippers with dedicated grasp detection.

        Returns:
            True if an object is detected in gripper
        """
        return self.is_closed() and self.get_force() > 0.5
