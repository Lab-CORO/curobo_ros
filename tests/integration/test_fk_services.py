#!/usr/bin/env python3
"""
Integration tests for curobo_fk node.

Tests the Forward Kinematics service that converts joint states
to end-effector poses.

Node: curobo_fk
Services tested:
- /curobo/fk_poses (curobo_msgs/srv/Fk)

Usage:
    pytest tests/integration/test_fk_services.py
"""

import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from curobo_msgs.srv import Fk

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from fixtures.test_poses import TestJointStates
from fixtures.test_robot_configs import TestRobotConfig


class TestFKServices:
    """Test suite for FK services."""

    @classmethod
    def setup_class(cls):
        """Initialize ROS2 for all tests."""
        rclpy.init()

    @classmethod
    def teardown_class(cls):
        """Shutdown ROS2 after all tests."""
        rclpy.shutdown()

    def setup_method(self):
        """Create test node before each test."""
        self.node = Node('test_fk_node')
        self.fk_client = self.node.create_client(
            Fk,
            '/curobo/fk_poses'
        )

    def teardown_method(self):
        """Destroy test node after each test."""
        self.node.destroy_node()

    def test_fk_service_availability(self):
        """Test that FK service is available."""
        assert self.fk_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT), \
            "FK service /curobo/fk_poses not available"

    def test_fk_single_joint_state(self):
        """Test FK for single joint state."""
        # Wait for service
        assert self.fk_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Create request
        request = Fk.Request()
        request.joint_states = [TestJointStates.home_state()]

        # Call service
        future = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Verify response
        assert future.result() is not None, "FK service call failed"
        response = future.result()
        assert len(response.poses) == 1, f"Expected 1 pose, got {len(response.poses)}"

        # Verify pose is valid
        pose = response.poses[0]
        assert pose.position is not None, "Pose position is None"
        assert pose.orientation is not None, "Pose orientation is None"

        # Check orientation is normalized
        quat = pose.orientation
        norm = (quat.w**2 + quat.x**2 + quat.y**2 + quat.z**2)**0.5
        assert abs(norm - 1.0) < 0.01, f"Quaternion not normalized: {norm}"

    def test_fk_batch_joint_states(self):
        """Test FK for batch of joint states."""
        # Wait for service
        assert self.fk_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Create batch request (10 joint states)
        batch_size = 10
        request = Fk.Request()
        request.joint_states = TestJointStates.batch_states(batch_size)

        # Call service
        future = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Verify response
        assert future.result() is not None, "FK batch service call failed"
        response = future.result()
        assert len(response.poses) == batch_size, \
            f"Expected {batch_size} poses, got {len(response.poses)}"

        # Verify all poses are valid
        for i, pose in enumerate(response.poses):
            assert pose.position is not None, f"Pose {i} position is None"
            assert pose.orientation is not None, f"Pose {i} orientation is None"

    def test_fk_different_configurations(self):
        """Test FK for different valid joint configurations."""
        # Wait for service
        assert self.fk_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Test multiple configurations
        test_states = [
            TestJointStates.home_state(),
            TestJointStates.valid_state_1(),
            TestJointStates.valid_state_2(),
        ]

        for i, joint_state in enumerate(test_states):
            request = Fk.Request()
            request.joint_states = [joint_state]

            # Call service
            future = self.fk_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

            # Verify response
            assert future.result() is not None, f"FK service call {i} failed"
            response = future.result()
            assert len(response.poses) == 1, f"Test {i}: Expected 1 pose"

    def test_fk_empty_request(self):
        """Test FK service with empty joint states."""
        # Wait for service
        assert self.fk_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Create empty request
        request = Fk.Request()
        request.joint_states = []

        # Call service
        future = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Service should handle empty request gracefully
        assert future.result() is not None, "FK service should respond to empty request"
        response = future.result()
        assert len(response.poses) == 0, "Expected empty poses list"

    def test_fk_large_batch(self):
        """Test FK service with large batch (100 states)."""
        # Wait for service
        assert self.fk_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Create large batch
        batch_size = 100
        request = Fk.Request()
        request.joint_states = TestJointStates.batch_states(batch_size)

        # Call service
        future = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2  # More time for large batch
        )

        # Verify response
        assert future.result() is not None, "FK large batch service call failed"
        response = future.result()
        assert len(response.poses) == batch_size, \
            f"Expected {batch_size} poses, got {len(response.poses)}"

    def test_fk_consistency(self):
        """Test that FK gives same result for same input."""
        # Wait for service
        assert self.fk_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Get FK result twice for same input
        request = Fk.Request()
        request.joint_states = [TestJointStates.home_state()]

        # First call
        future1 = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future1, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)
        result1 = future1.result()

        # Second call
        future2 = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future2, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)
        result2 = future2.result()

        # Verify both results exist
        assert result1 is not None and result2 is not None

        # Compare poses (should be identical)
        pose1 = result1.poses[0]
        pose2 = result2.poses[0]

        # Position should be same
        assert abs(pose1.position.x - pose2.position.x) < 1e-6
        assert abs(pose1.position.y - pose2.position.y) < 1e-6
        assert abs(pose1.position.z - pose2.position.z) < 1e-6

        # Orientation should be same
        assert abs(pose1.orientation.w - pose2.orientation.w) < 1e-6
        assert abs(pose1.orientation.x - pose2.orientation.x) < 1e-6
        assert abs(pose1.orientation.y - pose2.orientation.y) < 1e-6
        assert abs(pose1.orientation.z - pose2.orientation.z) < 1e-6


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
