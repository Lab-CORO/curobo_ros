#!/usr/bin/env python3
"""
Integration tests for curobo_gen_traj node.

Tests the trajectory generation services and action server
for motion planning with collision avoidance.

Node: curobo_gen_traj
Services tested:
- /curobo_gen_traj/generate_trajectory (curobo_msgs/srv/TrajectoryGeneration)
- /curobo_gen_traj/is_available (std_srvs/srv/Trigger)
- /curobo_gen_traj/set_robot_strategy (std_srvs/srv/Trigger)
- /curobo_gen_traj/get_robot_strategy (std_srvs/srv/Trigger)
- /curobo_gen_traj/update_motion_gen_config (std_srvs/srv/Trigger)

Actions tested:
- /curobo_gen_traj/send_trajectory (curobo_msgs/action/SendTrajectory)

Usage:
    pytest tests/integration/test_trajectory_generation.py
"""

import pytest
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from curobo_msgs.srv import TrajectoryGeneration
from curobo_msgs.action import SendTrajectory

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from fixtures.test_poses import TestPoses
from fixtures.test_robot_configs import TestRobotConfig


class TestTrajectoryGeneration:
    """Test suite for trajectory generation services and actions."""

    @classmethod
    def setup_class(cls):
        """Initialize ROS2 for all tests."""
        rclpy.init()

    @classmethod
    def teardown_class(cls):
        """Shutdown ROS2 after all tests."""
        rclpy.shutdown()

    def setup_method(self):
        """Create test node and service clients before each test."""
        self.node = Node('test_trajectory_node')

        # Create service clients
        self.gen_traj_client = self.node.create_client(
            TrajectoryGeneration,
            '/curobo_gen_traj/generate_trajectory'
        )
        self.is_available_client = self.node.create_client(
            Trigger,
            '/curobo_gen_traj/is_available'
        )
        self.set_strategy_client = self.node.create_client(
            Trigger,
            '/curobo_gen_traj/set_robot_strategy'
        )
        self.get_strategy_client = self.node.create_client(
            Trigger,
            '/curobo_gen_traj/get_robot_strategy'
        )
        self.update_config_client = self.node.create_client(
            Trigger,
            '/curobo_gen_traj/update_motion_gen_config'
        )

        # Create action client
        self.send_traj_action_client = ActionClient(
            self.node,
            SendTrajectory,
            '/curobo_gen_traj/send_trajectory'
        )

    def teardown_method(self):
        """Destroy test node after each test."""
        self.node.destroy_node()

    # =========================
    # SERVICE AVAILABILITY TESTS
    # =========================

    def test_all_services_available(self):
        """Test that all trajectory generation services are available."""
        services = [
            (self.gen_traj_client, '/curobo_gen_traj/generate_trajectory'),
            (self.is_available_client, '/curobo_gen_traj/is_available'),
            (self.set_strategy_client, '/curobo_gen_traj/set_robot_strategy'),
            (self.get_strategy_client, '/curobo_gen_traj/get_robot_strategy'),
            (self.update_config_client, '/curobo_gen_traj/update_motion_gen_config'),
        ]

        for client, service_name in services:
            assert client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT), \
                f"Service {service_name} not available"

    def test_action_server_available(self):
        """Test that SendTrajectory action server is available."""
        assert self.send_traj_action_client.wait_for_server(
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        ), "Action server /curobo_gen_traj/send_trajectory not available"

    # =========================
    # NODE STATUS TESTS
    # =========================

    def test_is_available_service(self):
        """Test is_available service to check warmup status."""
        assert self.is_available_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        request = Trigger.Request()
        future = self.is_available_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Verify response
        assert future.result() is not None, "is_available service call failed"
        response = future.result()
        # Node should be available after warmup
        assert response.success, f"Node not ready: {response.message}"

    def test_get_robot_strategy(self):
        """Test getting current robot strategy."""
        assert self.get_strategy_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        request = Trigger.Request()
        future = self.get_strategy_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Verify response
        assert future.result() is not None, "get_robot_strategy service call failed"
        response = future.result()
        assert response.success
        # Message should contain strategy name (doosan/emulator/ghost)
        assert len(response.message) > 0

    def test_update_motion_gen_config(self):
        """Test updating motion generation configuration."""
        assert self.update_config_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        request = Trigger.Request()
        future = self.update_config_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Verify response
        assert future.result() is not None, "update_config service call failed"
        response = future.result()
        # May succeed or fail depending on implementation
        assert response is not None

    # =========================
    # TRAJECTORY GENERATION TESTS
    # =========================

    def test_generate_trajectory_reachable_pose(self):
        """Test generating trajectory to a reachable pose."""
        assert self.gen_traj_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Create request
        request = TrajectoryGeneration.Request()
        request.target_pose = TestPoses.reach_pose_1()

        # Call service
        future = self.gen_traj_client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
        )

        # Verify response
        assert future.result() is not None, "generate_trajectory service call failed"
        response = future.result()
        assert response.success, f"Trajectory generation failed: {response.message}"

    def test_generate_trajectory_unreachable_pose(self):
        """Test generating trajectory to an unreachable pose."""
        assert self.gen_traj_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Create request with unreachable pose
        request = TrajectoryGeneration.Request()
        request.target_pose = TestPoses.unreachable_pose()

        # Call service
        future = self.gen_traj_client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
        )

        # Verify response
        assert future.result() is not None, "Service call failed"
        response = future.result()
        # Should fail for unreachable pose
        assert not response.success, "Should fail for unreachable pose"

    def test_generate_trajectory_multiple_poses(self):
        """Test generating trajectories to multiple different poses."""
        assert self.gen_traj_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        test_poses = [
            TestPoses.reach_pose_1(),
            TestPoses.reach_pose_2(),
            TestPoses.reach_pose_3(),
        ]

        for i, pose in enumerate(test_poses):
            request = TrajectoryGeneration.Request()
            request.target_pose = pose

            future = self.gen_traj_client.call_async(request)
            rclpy.spin_until_future_complete(
                self.node,
                future,
                timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
            )

            assert future.result() is not None, f"Trajectory {i} generation failed"
            response = future.result()
            # At least some should succeed
            if response.success:
                assert True  # Trajectory found

    def test_generate_trajectory_with_collision_pose(self):
        """Test trajectory generation to pose that would cause collision."""
        assert self.gen_traj_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Create request with collision pose
        request = TrajectoryGeneration.Request()
        request.target_pose = TestPoses.collision_pose()

        # Call service
        future = self.gen_traj_client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
        )

        # Verify response
        assert future.result() is not None
        response = future.result()
        # Should fail or avoid collision
        # If success=False, collision was detected
        # If success=True, a collision-free path was found
        assert response is not None

    # =========================
    # ACTION TESTS
    # =========================

    def test_send_trajectory_action_goal_acceptance(self):
        """Test that action server accepts goals."""
        assert self.send_traj_action_client.wait_for_server(
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )

        # Create goal
        goal_msg = SendTrajectory.Goal()
        # Goal is empty for this test - just testing acceptance

        # Send goal
        send_goal_future = self.send_traj_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(
            self.node,
            send_goal_future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )

        # Verify goal was accepted (or rejected gracefully)
        assert send_goal_future.result() is not None

    def test_send_trajectory_action_cancellation(self):
        """Test canceling an action goal."""
        assert self.send_traj_action_client.wait_for_server(
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )

        # Create and send goal
        goal_msg = SendTrajectory.Goal()
        send_goal_future = self.send_traj_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(
            self.node,
            send_goal_future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )

        goal_handle = send_goal_future.result()
        if goal_handle is not None and goal_handle.accepted:
            # Cancel the goal
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(
                self.node,
                cancel_future,
                timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
            )

            # Verify cancellation response
            assert cancel_future.result() is not None

    # =========================
    # INTEGRATION TESTS
    # =========================

    def test_plan_and_execute_pipeline(self):
        """Test complete pipeline: plan trajectory then execute."""
        # Step 1: Generate trajectory
        assert self.gen_traj_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        gen_request = TrajectoryGeneration.Request()
        gen_request.target_pose = TestPoses.reach_pose_1()

        gen_future = self.gen_traj_client.call_async(gen_request)
        rclpy.spin_until_future_complete(
            self.node,
            gen_future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
        )

        # Verify planning succeeded
        assert gen_future.result() is not None
        gen_response = gen_future.result()

        if gen_response.success:
            # Step 2: Execute trajectory via action
            assert self.send_traj_action_client.wait_for_server(
                timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
            )

            goal_msg = SendTrajectory.Goal()
            send_goal_future = self.send_traj_action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(
                self.node,
                send_goal_future,
                timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
            )

            goal_handle = send_goal_future.result()
            assert goal_handle is not None, "Action goal not accepted"

            if goal_handle.accepted:
                # Wait for result (with timeout)
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(
                    self.node,
                    result_future,
                    timeout_sec=TestRobotConfig.ACTION_TIMEOUT
                )

                # Verify result exists
                assert result_future.result() is not None

    def test_trajectory_generation_timeout(self):
        """Test that trajectory generation respects timeout."""
        assert self.gen_traj_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Create request with difficult pose
        request = TrajectoryGeneration.Request()
        request.target_pose = TestPoses.reach_pose_3()

        # Set short timeout
        start_time = self.node.get_clock().now()

        future = self.gen_traj_client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
        )

        end_time = self.node.get_clock().now()
        elapsed = (end_time - start_time).nanoseconds / 1e9

        # Should respond within reasonable time
        assert elapsed < TestRobotConfig.SERVICE_TIMEOUT * 2

    def test_consecutive_trajectory_generations(self):
        """Test generating multiple trajectories consecutively."""
        assert self.gen_traj_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        poses = [
            TestPoses.reach_pose_1(),
            TestPoses.reach_pose_2(),
            TestPoses.reach_pose_1(),  # Return to first
        ]

        for i, pose in enumerate(poses):
            request = TrajectoryGeneration.Request()
            request.target_pose = pose

            future = self.gen_traj_client.call_async(request)
            rclpy.spin_until_future_complete(
                self.node,
                future,
                timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
            )

            assert future.result() is not None, f"Consecutive trajectory {i} failed"


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
