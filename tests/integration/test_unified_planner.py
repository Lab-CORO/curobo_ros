#!/usr/bin/env python3
"""
Integration tests for unified_planner node (MPC branch).

Tests the unified planner node that supports multiple planning
strategies: Classic, MPC, Batch, and Constrained planners.

Node: unified_planner (curobo_trajectory_planner)
Services tested:
- /unified_planner/generate_trajectory (curobo_msgs/srv/TrajectoryGeneration)
- /unified_planner/set_planner (curobo_msgs/srv/SetPlanner)
- /unified_planner/list_planners (std_srvs/srv/Trigger)

Actions tested:
- /unified_planner/execute_trajectory (curobo_msgs/action/SendTrajectory)

Usage:
    pytest tests/integration/test_unified_planner.py
"""

import pytest
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from curobo_msgs.srv import TrajectoryGeneration, SetPlanner
from curobo_msgs.action import SendTrajectory

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from fixtures.test_poses import TestPoses
from fixtures.test_robot_configs import TestRobotConfig


class TestUnifiedPlanner:
    """Test suite for unified_planner node."""

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
        self.node = Node('test_unified_planner_node')

        # Create service clients
        self.gen_traj_client = self.node.create_client(
            TrajectoryGeneration,
            '/unified_planner/generate_trajectory'
        )
        self.set_planner_client = self.node.create_client(
            SetPlanner,
            '/unified_planner/set_planner'
        )
        self.list_planners_client = self.node.create_client(
            Trigger,
            '/unified_planner/list_planners'
        )

        # Create action client
        self.execute_action_client = ActionClient(
            self.node,
            SendTrajectory,
            '/unified_planner/execute_trajectory'
        )

    def teardown_method(self):
        """Destroy test node after each test."""
        self.node.destroy_node()

    # =========================
    # SERVICE AVAILABILITY TESTS
    # =========================

    def test_all_services_available(self):
        """Test that all unified planner services are available."""
        services = [
            (self.gen_traj_client, '/unified_planner/generate_trajectory'),
            (self.set_planner_client, '/unified_planner/set_planner'),
            (self.list_planners_client, '/unified_planner/list_planners'),
        ]

        for client, service_name in services:
            assert client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT), \
                f"Service {service_name} not available"

    def test_action_server_available(self):
        """Test that execute_trajectory action server is available."""
        assert self.execute_action_client.wait_for_server(
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        ), "Action server /unified_planner/execute_trajectory not available"

    # =========================
    # PLANNER LISTING TESTS
    # =========================

    def test_list_planners_service(self):
        """Test listing available planners."""
        assert self.list_planners_client.wait_for_service(
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )

        request = Trigger.Request()
        future = self.list_planners_client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )

        # Verify response
        assert future.result() is not None, "list_planners service call failed"
        response = future.result()
        assert response.success, "list_planners should succeed"
        assert len(response.message) > 0, "Planner list should not be empty"

        # Should contain planner types
        message = response.message.lower()
        assert 'classic' in message or 'mpc' in message, \
            "Should list at least classic or mpc planner"

    def test_list_planners_shows_current(self):
        """Test that list_planners shows current active planner."""
        assert self.list_planners_client.wait_for_service(
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )

        request = Trigger.Request()
        future = self.list_planners_client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )

        response = future.result()
        assert response.success
        # Message should indicate current planner
        assert 'current' in response.message.lower()

    # =========================
    # PLANNER SWITCHING TESTS
    # =========================

    def test_set_planner_classic(self):
        """Test switching to classic planner."""
        assert self.set_planner_client.wait_for_service(
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )

        # Set to classic planner (enum value 0)
        request = SetPlanner.Request()
        request.planner_type = SetPlanner.Request.CLASSIC

        future = self.set_planner_client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )

        # Verify response
        assert future.result() is not None, "set_planner service call failed"
        response = future.result()
        assert response.success, f"Failed to set classic planner: {response.message}"
        assert 'classic' in response.current_planner.lower(), \
            "Current planner should be classic"

    def test_set_planner_mpc(self):
        """Test switching to MPC planner."""
        assert self.set_planner_client.wait_for_service(
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )

        # Set to MPC planner (enum value 1)
        request = SetPlanner.Request()
        request.planner_type = SetPlanner.Request.MPC

        future = self.set_planner_client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2  # MPC may need warmup
        )

        # Verify response
        assert future.result() is not None, "set_planner service call failed"
        response = future.result()
        assert response.success, f"Failed to set MPC planner: {response.message}"
        assert 'mpc' in response.current_planner.lower(), \
            "Current planner should be MPC"

    def test_set_planner_invalid(self):
        """Test setting invalid planner type."""
        assert self.set_planner_client.wait_for_service(
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )

        # Use invalid enum value (999)
        request = SetPlanner.Request()
        request.planner_type = 999

        future = self.set_planner_client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )

        # Verify response
        assert future.result() is not None
        response = future.result()
        # Should fail for invalid planner
        assert not response.success, "Should reject invalid planner type"

    def test_set_planner_returns_previous(self):
        """Test that set_planner returns previous planner name."""
        assert self.set_planner_client.wait_for_service(
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )

        # Switch to classic
        request = SetPlanner.Request()
        request.planner_type = SetPlanner.Request.CLASSIC
        future = self.set_planner_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)
        response1 = future.result()

        # Switch to MPC
        request.planner_type = SetPlanner.Request.MPC
        future = self.set_planner_client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
        )
        response2 = future.result()

        if response1.success and response2.success:
            # Previous planner should be classic
            assert 'classic' in response2.previous_planner.lower(), \
                "Previous planner should be classic"
            # Current should be MPC
            assert 'mpc' in response2.current_planner.lower(), \
                "Current planner should be MPC"

    # =========================
    # TRAJECTORY GENERATION TESTS
    # =========================

    def test_generate_trajectory_with_classic(self):
        """Test generating trajectory with classic planner."""
        # Set to classic planner
        assert self.set_planner_client.wait_for_service(
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )
        set_req = SetPlanner.Request()
        set_req.planner_type = SetPlanner.Request.CLASSIC
        future = self.set_planner_client.call_async(set_req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Generate trajectory
        assert self.gen_traj_client.wait_for_service(
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )

        traj_req = TrajectoryGeneration.Request()
        traj_req.target_pose = TestPoses.reach_pose_1()

        future = self.gen_traj_client.call_async(traj_req)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
        )

        # Verify response
        assert future.result() is not None, "Trajectory generation failed"
        response = future.result()
        # May succeed or fail depending on pose reachability
        assert response is not None

    def test_generate_trajectory_with_mpc(self):
        """Test generating trajectory with MPC planner."""
        # Set to MPC planner
        assert self.set_planner_client.wait_for_service(
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )
        set_req = SetPlanner.Request()
        set_req.planner_type = SetPlanner.Request.MPC
        future = self.set_planner_client.call_async(set_req)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
        )

        # Generate trajectory (MPC sets up goal buffer)
        assert self.gen_traj_client.wait_for_service(
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )

        traj_req = TrajectoryGeneration.Request()
        traj_req.target_pose = TestPoses.reach_pose_1()

        future = self.gen_traj_client.call_async(traj_req)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
        )

        # Verify response
        assert future.result() is not None, "MPC trajectory generation failed"
        response = future.result()
        # MPC planning should succeed (just sets up goal buffer)
        assert response is not None

    def test_generate_multiple_trajectories(self):
        """Test generating multiple consecutive trajectories."""
        assert self.gen_traj_client.wait_for_service(
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )

        poses = [
            TestPoses.reach_pose_1(),
            TestPoses.reach_pose_2(),
            TestPoses.reach_pose_3(),
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

            assert future.result() is not None, f"Trajectory {i} generation failed"

    # =========================
    # ACTION TESTS
    # =========================

    def test_execute_action_goal_acceptance(self):
        """Test that execute action accepts goals."""
        assert self.execute_action_client.wait_for_server(
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )

        # Create goal
        goal_msg = SendTrajectory.Goal()

        # Send goal
        send_goal_future = self.execute_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(
            self.node,
            send_goal_future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )

        # Verify goal handling
        assert send_goal_future.result() is not None

    def test_execute_action_cancellation(self):
        """Test canceling execution action."""
        assert self.execute_action_client.wait_for_server(
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )

        # Create and send goal
        goal_msg = SendTrajectory.Goal()
        send_goal_future = self.execute_action_client.send_goal_async(goal_msg)
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

            # Verify cancellation
            assert cancel_future.result() is not None

    # =========================
    # INTEGRATION TESTS
    # =========================

    def test_plan_and_execute_classic(self):
        """Test complete pipeline with classic planner: plan → execute."""
        # Set to classic
        assert self.set_planner_client.wait_for_service(
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )
        set_req = SetPlanner.Request()
        set_req.planner_type = SetPlanner.Request.CLASSIC
        future = self.set_planner_client.call_async(set_req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Plan
        assert self.gen_traj_client.wait_for_service(
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )
        traj_req = TrajectoryGeneration.Request()
        traj_req.target_pose = TestPoses.reach_pose_1()

        future = self.gen_traj_client.call_async(traj_req)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
        )

        plan_response = future.result()
        if plan_response and plan_response.success:
            # Execute
            assert self.execute_action_client.wait_for_server(
                timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
            )

            goal_msg = SendTrajectory.Goal()
            send_goal_future = self.execute_action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(
                self.node,
                send_goal_future,
                timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
            )

            goal_handle = send_goal_future.result()
            assert goal_handle is not None

            if goal_handle.accepted:
                # Wait for result (with timeout)
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(
                    self.node,
                    result_future,
                    timeout_sec=TestRobotConfig.ACTION_TIMEOUT
                )

                assert result_future.result() is not None

    def test_switch_planner_between_plans(self):
        """Test switching planner between trajectory generations."""
        assert self.set_planner_client.wait_for_service(
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )
        assert self.gen_traj_client.wait_for_service(
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )

        # Plan with classic
        set_req = SetPlanner.Request()
        set_req.planner_type = SetPlanner.Request.CLASSIC
        future = self.set_planner_client.call_async(set_req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        traj_req = TrajectoryGeneration.Request()
        traj_req.target_pose = TestPoses.reach_pose_1()
        future = self.gen_traj_client.call_async(traj_req)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
        )

        # Switch to MPC
        set_req.planner_type = SetPlanner.Request.MPC
        future = self.set_planner_client.call_async(set_req)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
        )

        # Plan with MPC
        traj_req.target_pose = TestPoses.reach_pose_2()
        future = self.gen_traj_client.call_async(traj_req)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
        )

        # Both should complete without errors
        assert future.result() is not None


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
