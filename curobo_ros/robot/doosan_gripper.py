#!/usr/bin/env python3
"""
Doosan M1013 gripper interface.

Implements gripper control for the Doosan M1013 robot arm.
Adjust topic names and message types based on your Doosan ROS2 interface.
"""

import time
from typing import Optional

import rclpy
from std_msgs.msg import Float64

from .gripper_interface import GripperInterface


class DoosanGripper(GripperInterface):
    """
    Gripper interface for Doosan M1013.

    This implementation assumes a simple Float64 command interface.
    Adjust the topic names and message types based on your actual
    Doosan ROS2 driver implementation.

    Topic configuration:
    - Command: /doosan/gripper/command (std_msgs/Float64)
        - 0.0 = fully closed
        - 1.0 = fully open

    Parameters:
    - grasp_gripper_close_time: Time to wait for gripper operation (seconds)

    Note:
        If your Doosan driver uses different topics or message types
        (e.g., JointState, custom Doosan messages), modify accordingly.
    """

    def __init__(self, node):
        """
        Initialize Doosan gripper interface.

        Args:
            node: ROS2 node for logging and parameter access
        """
        self.node = node
        self.current_state = 1.0  # Start fully open

        # Create publisher for gripper commands
        # TODO: Verify this topic name with your Doosan ROS2 driver
        self.gripper_pub = node.create_publisher(
            Float64,
            '/doosan/gripper/command',
            10
        )

        # Optional: Subscribe to gripper state feedback
        # If your driver provides feedback, uncomment and implement callback
        # self.gripper_sub = node.create_subscription(
        #     Float64,
        #     '/doosan/gripper/state',
        #     self._gripper_state_callback,
        #     10
        # )

        self.node.get_logger().info("DoosanGripper initialized")

    def open(self) -> bool:
        """
        Open Doosan gripper.

        Returns:
            True if successful
        """
        msg = Float64()
        msg.data = 1.0  # Fully open

        self.gripper_pub.publish(msg)
        self.node.get_logger().info("Opening Doosan gripper")

        # Wait for gripper to open
        gripper_time = self.node.get_parameter(
            'grasp_gripper_close_time'
        ).get_parameter_value().double_value
        time.sleep(gripper_time)

        self.current_state = 1.0
        return True

    def close(self) -> bool:
        """
        Close Doosan gripper.

        Returns:
            True if successful
        """
        msg = Float64()
        msg.data = 0.0  # Fully closed

        self.gripper_pub.publish(msg)
        self.node.get_logger().info("Closing Doosan gripper")

        # Wait for gripper to close
        gripper_time = self.node.get_parameter(
            'grasp_gripper_close_time'
        ).get_parameter_value().double_value
        time.sleep(gripper_time)

        self.current_state = 0.0
        return True

    def get_state(self) -> float:
        """
        Get current gripper position.

        Returns:
            Position (0.0 = closed, 1.0 = open)

        Note:
            This implementation returns the commanded state.
            If feedback is available, implement _gripper_state_callback()
            to update current_state from actual sensor readings.
        """
        return self.current_state

    def set_position(self, position: float) -> bool:
        """
        Set gripper to specific position.

        Args:
            position: Target position (0.0 = closed, 1.0 = open)

        Returns:
            True if successful
        """
        # Clamp position to valid range
        position = max(0.0, min(1.0, position))

        msg = Float64()
        msg.data = position

        self.gripper_pub.publish(msg)
        self.node.get_logger().info(f"Setting Doosan gripper position to {position:.2f}")

        # Wait for gripper to move
        gripper_time = self.node.get_parameter(
            'grasp_gripper_close_time'
        ).get_parameter_value().double_value
        time.sleep(gripper_time)

        self.current_state = position
        return True

    def _gripper_state_callback(self, msg: Float64):
        """
        Callback for gripper state feedback (if available).

        Args:
            msg: Gripper state message

        Note:
            Uncomment subscriber in __init__ to use this.
        """
        self.current_state = msg.data
