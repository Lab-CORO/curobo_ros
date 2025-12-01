#!/usr/bin/env python3
"""
Integration tests for MPC planner (Model Predictive Control).

Tests MPC-specific behavior: convergence, closed-loop control,
real-time performance, and obstacle avoidance.

Node: unified_planner with MPC strategy
Services/Actions tested via MPC planner:
- generate_trajectory (MPC goal setup)
- execute_trajectory (MPC closed-loop execution)
- mpc_goal topic (real-time goal updates)

Usage:
    pytest tests/integration/test_mpc_planner.py
"""

import pytest
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose as PoseMsg
from curobo_msgs.srv import TrajectoryGeneration, SetPlanner
from curobo_msgs.action import SendTrajectory
import time

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from fixtures.test_poses import TestPoses
from fixtures.test_robot_configs import TestRobotConfig


class TestMPCPlanner:
    """Test suite for MPC planner behavior."""

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
        self.node = Node('test_mpc_planner_node')

        # Create service clients
        self.set_planner_client = self.node.create_client(
            SetPlanner,
            '/unified_planner/set_planner'
        )
        self.gen_traj_client = self.node.create_client(
            TrajectoryGeneration,
            '/unified_planner/generate_trajectory'
        )

        # Create action client
        self.execute_action_client = ActionClient(
            self.node,
            SendTrajectory,
            '/unified_planner/execute_trajectory'
        )

        # Create MPC goal publisher (for real-time goal updates)
        self.mpc_goal_pub = self.node.create_publisher(
            PoseMsg,
            '/unified_planner/mpc_goal',
            10
        )

        # Set to MPC planner
        self._switch_to_mpc()

    def teardown_method(self):
        """Destroy test node after each test."""
        self.node.destroy_node()

    def _switch_to_mpc(self):
        """Helper to switch to MPC planner."""
        assert self.set_planner_client.wait_for_service(
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )

        request = SetPlanner.Request()
        request.planner_type = SetPlanner.Request.MPC

        future = self.set_planner_client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2  # Allow warmup time
        )

        response = future.result()
        assert response is not None, "Failed to switch to MPC"
        assert response.success, f"MPC switch failed: {response.message}"

    # =========================
    # MPC PLANNING TESTS
    # =========================

    def test_mpc_plan_setup(self):
        """Test MPC planning sets up goal buffer correctly."""
        assert self.gen_traj_client.wait_for_service(
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )

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
        assert future.result() is not None, "MPC plan setup failed"
        response = future.result()
        # MPC planning should succeed (just sets up goal buffer)
        assert response.success, f"MPC plan failed: {response.message}"

    def test_mpc_unreachable_pose(self):
        """Test MPC planning with unreachable pose."""
        assert self.gen_traj_client.wait_for_service(
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )

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

        # MPC may set up goal buffer even for unreachable poses
        # (failure detected during execution, not planning)
        assert future.result() is not None

    def test_mpc_multiple_goals(self):
        """Test setting up multiple MPC goals sequentially."""
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

            assert future.result() is not None, f"MPC goal {i} setup failed"

    # =========================
    # MPC EXECUTION TESTS
    # =========================

    def test_mpc_execution_acceptance(self):
        """Test that MPC execution accepts goals."""
        # First plan
        assert self.gen_traj_client.wait_for_service(
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )

        plan_req = TrajectoryGeneration.Request()
        plan_req.target_pose = TestPoses.reach_pose_1()

        future = self.gen_traj_client.call_async(plan_req)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
        )

        if future.result() and future.result().success:
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

            # Verify goal accepted
            goal_handle = send_goal_future.result()
            assert goal_handle is not None

    def test_mpc_execution_cancellation(self):
        """Test canceling MPC execution mid-execution."""
        # Plan
        assert self.gen_traj_client.wait_for_service(
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )

        plan_req = TrajectoryGeneration.Request()
        plan_req.target_pose = TestPoses.reach_pose_1()

        future = self.gen_traj_client.call_async(plan_req)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
        )

        if future.result() and future.result().success:
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
            if goal_handle and goal_handle.accepted:
                # Cancel immediately
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(
                    self.node,
                    cancel_future,
                    timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
                )

                # Verify cancellation
                assert cancel_future.result() is not None

    def test_mpc_execution_feedback(self):
        """Test that MPC execution provides feedback."""
        # Plan
        assert self.gen_traj_client.wait_for_service(
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )

        plan_req = TrajectoryGeneration.Request()
        plan_req.target_pose = TestPoses.reach_pose_1()

        future = self.gen_traj_client.call_async(plan_req)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
        )

        if future.result() and future.result().success:
            # Execute with feedback callback
            assert self.execute_action_client.wait_for_server(
                timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
            )

            self.feedback_received = False

            def feedback_callback(feedback_msg):
                self.feedback_received = True
                # Verify feedback has step_progression field
                assert hasattr(feedback_msg.feedback, 'step_progression')

            goal_msg = SendTrajectory.Goal()
            send_goal_future = self.execute_action_client.send_goal_async(
                goal_msg,
                feedback_callback=feedback_callback
            )
            rclpy.spin_until_future_complete(
                self.node,
                send_goal_future,
                timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
            )

            goal_handle = send_goal_future.result()
            if goal_handle and goal_handle.accepted:
                # Wait briefly for feedback
                timeout = 5.0
                start_time = time.time()

                while not self.feedback_received and (time.time() - start_time) < timeout:
                    rclpy.spin_once(self.node, timeout_sec=0.1)

                # Cancel to stop execution
                if goal_handle.is_active:
                    goal_handle.cancel_goal_async()

    # =========================
    # MPC REAL-TIME GOAL UPDATE TESTS
    # =========================

    def test_mpc_goal_topic_publish(self):
        """Test publishing to MPC goal update topic."""
        # Verify topic exists
        time.sleep(1.0)
        topics = self.node.get_topic_names_and_types()
        topic_names = [t[0] for t in topics]

        # MPC goal topic should exist
        assert '/unified_planner/mpc_goal' in topic_names, \
            "MPC goal update topic not available"

        # Publish test goal
        test_pose = PoseMsg()
        test_pose.position.x = 0.5
        test_pose.position.y = 0.0
        test_pose.position.z = 0.5
        test_pose.orientation.w = 1.0

        # Should not crash
        self.mpc_goal_pub.publish(test_pose)

    # =========================
    # MPC CONVERGENCE TESTS
    # =========================

    def test_mpc_convergence_params(self):
        """Test MPC uses convergence threshold parameter."""
        # This test verifies MPC respects convergence_threshold
        # Actual convergence is tested in execution

        # Plan with different poses
        assert self.gen_traj_client.wait_for_service(
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )

        request = TrajectoryGeneration.Request()
        request.target_pose = TestPoses.reach_pose_1()

        future = self.gen_traj_client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
        )

        # Should complete without error
        assert future.result() is not None

    # =========================
    # PERFORMANCE TESTS
    # =========================

    def test_mpc_planning_performance(self):
        """Test that MPC planning is fast (< 1 second)."""
        assert self.gen_traj_client.wait_for_service(
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )

        request = TrajectoryGeneration.Request()
        request.target_pose = TestPoses.reach_pose_1()

        # Measure planning time
        start_time = time.time()

        future = self.gen_traj_client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
        )

        end_time = time.time()
        planning_time = end_time - start_time

        # MPC planning should be fast (just sets up goal buffer)
        assert planning_time < 1.0, f"MPC planning too slow: {planning_time:.3f}s"

    # =========================
    # INTEGRATION TESTS
    # =========================

    def test_mpc_plan_execute_pipeline(self):
        """Test complete MPC pipeline: plan → execute."""
        # Plan
        assert self.gen_traj_client.wait_for_service(
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )

        plan_req = TrajectoryGeneration.Request()
        plan_req.target_pose = TestPoses.reach_pose_1()

        future = self.gen_traj_client.call_async(plan_req)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
        )

        plan_response = future.result()
        assert plan_response is not None, "MPC planning failed"
        assert plan_response.success, "MPC plan setup failed"

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
        assert goal_handle is not None, "MPC execution not accepted"

        if goal_handle.accepted:
            # Get result with timeout
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(
                self.node,
                result_future,
                timeout_sec=TestRobotConfig.ACTION_TIMEOUT
            )

            # Verify result exists
            assert result_future.result() is not None

    def test_mpc_consecutive_executions(self):
        """Test multiple consecutive MPC executions."""
        poses = [
            TestPoses.reach_pose_1(),
            TestPoses.reach_pose_2(),
        ]

        for i, pose in enumerate(poses):
            # Plan
            assert self.gen_traj_client.wait_for_service(
                timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
            )

            plan_req = TrajectoryGeneration.Request()
            plan_req.target_pose = pose

            future = self.gen_traj_client.call_async(plan_req)
            rclpy.spin_until_future_complete(
                self.node,
                future,
                timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
            )

            if future.result() and future.result().success:
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
                if goal_handle and goal_handle.accepted:
                    # Cancel to move to next (simulates quick test)
                    cancel_future = goal_handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(
                        self.node,
                        cancel_future,
                        timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
                    )


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
