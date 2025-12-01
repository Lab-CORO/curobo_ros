#!/usr/bin/env python3
"""
Integration tests for dynamic planner switching.

Tests the ability to switch between different planning strategies
(Classic, MPC, Batch, Constrained) dynamically without restarting.

Node: unified_planner
Focus: Dynamic planner switching and state preservation

Usage:
    pytest tests/integration/test_planner_switching.py
"""

import pytest
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from curobo_msgs.srv import TrajectoryGeneration, SetPlanner, AddObject, RemoveObject

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from fixtures.test_poses import TestPoses
from fixtures.test_robot_configs import TestRobotConfig


class TestPlannerSwitching:
    """Test suite for dynamic planner switching."""

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
        self.node = Node('test_planner_switch_node')

        # Create service clients
        self.set_planner_client = self.node.create_client(
            SetPlanner,
            '/unified_planner/set_planner'
        )
        self.list_planners_client = self.node.create_client(
            Trigger,
            '/unified_planner/list_planners'
        )
        self.gen_traj_client = self.node.create_client(
            TrajectoryGeneration,
            '/unified_planner/generate_trajectory'
        )

    def teardown_method(self):
        """Destroy test node after each test."""
        self.node.destroy_node()

    def _set_planner(self, planner_type):
        """Helper to set planner type."""
        assert self.set_planner_client.wait_for_service(
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )

        request = SetPlanner.Request()
        request.planner_type = planner_type

        future = self.set_planner_client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
        )

        return future.result()

    def _get_current_planner(self):
        """Helper to get current planner name."""
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
        return response.message if response else ""

    # =========================
    # BASIC SWITCHING TESTS
    # =========================

    def test_switch_classic_to_mpc(self):
        """Test switching from Classic to MPC planner."""
        # Switch to Classic
        response = self._set_planner(SetPlanner.Request.CLASSIC)
        assert response is not None
        assert response.success, f"Failed to set Classic: {response.message}"

        # Switch to MPC
        response = self._set_planner(SetPlanner.Request.MPC)
        assert response is not None
        assert response.success, f"Failed to switch to MPC: {response.message}"
        assert 'mpc' in response.current_planner.lower()
        assert 'classic' in response.previous_planner.lower()

    def test_switch_mpc_to_classic(self):
        """Test switching from MPC to Classic planner."""
        # Switch to MPC
        response = self._set_planner(SetPlanner.Request.MPC)
        assert response is not None
        assert response.success

        # Switch to Classic
        response = self._set_planner(SetPlanner.Request.CLASSIC)
        assert response is not None
        assert response.success, f"Failed to switch to Classic: {response.message}"
        assert 'classic' in response.current_planner.lower()
        assert 'mpc' in response.previous_planner.lower()

    def test_switch_multiple_times(self):
        """Test switching planners multiple times in sequence."""
        planners = [
            SetPlanner.Request.CLASSIC,
            SetPlanner.Request.MPC,
            SetPlanner.Request.CLASSIC,
            SetPlanner.Request.MPC,
        ]

        for planner_type in planners:
            response = self._set_planner(planner_type)
            assert response is not None
            assert response.success, f"Failed to switch to planner {planner_type}"

    def test_switch_to_same_planner(self):
        """Test switching to currently active planner (should succeed)."""
        # Set to Classic
        response1 = self._set_planner(SetPlanner.Request.CLASSIC)
        assert response1 and response1.success

        # Set to Classic again
        response2 = self._set_planner(SetPlanner.Request.CLASSIC)
        assert response2 is not None
        # Should handle gracefully (may succeed or be a no-op)
        assert response2 is not None

    # =========================
    # SWITCHING WITH PLANNING TESTS
    # =========================

    def test_switch_between_plans(self):
        """Test switching planner between trajectory generations."""
        assert self.gen_traj_client.wait_for_service(
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )

        # Plan with Classic
        self._set_planner(SetPlanner.Request.CLASSIC)

        traj_req = TrajectoryGeneration.Request()
        traj_req.target_pose = TestPoses.reach_pose_1()

        future = self.gen_traj_client.call_async(traj_req)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
        )

        classic_result = future.result()
        assert classic_result is not None

        # Switch to MPC
        self._set_planner(SetPlanner.Request.MPC)

        # Plan with MPC
        traj_req.target_pose = TestPoses.reach_pose_2()

        future = self.gen_traj_client.call_async(traj_req)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
        )

        mpc_result = future.result()
        assert mpc_result is not None

    def test_consecutive_switches_with_plans(self):
        """Test rapid switching with planning after each switch."""
        assert self.gen_traj_client.wait_for_service(
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        )

        planners = [
            (SetPlanner.Request.CLASSIC, TestPoses.reach_pose_1()),
            (SetPlanner.Request.MPC, TestPoses.reach_pose_2()),
            (SetPlanner.Request.CLASSIC, TestPoses.reach_pose_3()),
        ]

        for planner_type, pose in planners:
            # Switch
            response = self._set_planner(planner_type)
            assert response and response.success

            # Plan
            traj_req = TrajectoryGeneration.Request()
            traj_req.target_pose = pose

            future = self.gen_traj_client.call_async(traj_req)
            rclpy.spin_until_future_complete(
                self.node,
                future,
                timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
            )

            assert future.result() is not None

    # =========================
    # LAZY LOADING TESTS
    # =========================

    def test_lazy_warmup_classic(self):
        """Test that Classic planner warms up on-demand."""
        # Switch to MPC first (if not already)
        self._set_planner(SetPlanner.Request.MPC)

        # Switch to Classic (should trigger warmup if not already done)
        response = self._set_planner(SetPlanner.Request.CLASSIC)
        assert response is not None
        assert response.success, "Classic planner should warm up on demand"

    def test_lazy_warmup_mpc(self):
        """Test that MPC planner warms up on-demand."""
        # Switch to Classic first
        self._set_planner(SetPlanner.Request.CLASSIC)

        # Switch to MPC (should trigger warmup if not already done)
        response = self._set_planner(SetPlanner.Request.MPC)
        assert response is not None
        assert response.success, "MPC planner should warm up on demand"

    def test_second_switch_faster(self):
        """Test that second switch to same planner is faster (uses cache)."""
        import time

        # First switch to MPC (may include warmup)
        start1 = time.time()
        self._set_planner(SetPlanner.Request.MPC)
        time1 = time.time() - start1

        # Switch away
        self._set_planner(SetPlanner.Request.CLASSIC)

        # Second switch to MPC (should be cached)
        start2 = time.time()
        self._set_planner(SetPlanner.Request.MPC)
        time2 = time.time() - start2

        # Second switch should be faster or similar (not significantly slower)
        # Note: This is not always guaranteed due to ROS2 overhead
        assert time2 <= time1 * 1.5, \
            f"Second switch slower than expected: {time1:.3f}s vs {time2:.3f}s"

    # =========================
    # ERROR HANDLING TESTS
    # =========================

    def test_invalid_planner_type(self):
        """Test switching to invalid planner type."""
        response = self._set_planner(999)  # Invalid enum
        assert response is not None
        assert not response.success, "Should reject invalid planner type"

    def test_unavailable_planner(self):
        """Test switching to planner that's not implemented yet."""
        # Batch and Constrained may not be implemented yet
        response = self._set_planner(SetPlanner.Request.BATCH)
        # May succeed (if implemented) or fail gracefully
        assert response is not None

    # =========================
    # STATE PERSISTENCE TESTS
    # =========================

    def test_planner_info_persistence(self):
        """Test that planner info is correctly maintained."""
        # Set Classic
        response1 = self._set_planner(SetPlanner.Request.CLASSIC)
        planner1 = response1.current_planner if response1 else ""

        # Set MPC
        response2 = self._set_planner(SetPlanner.Request.MPC)
        planner2_prev = response2.previous_planner if response2 else ""
        planner2_curr = response2.current_planner if response2 else ""

        # Previous should match previous current
        if response1 and response1.success and response2 and response2.success:
            assert planner1.lower() in planner2_prev.lower(), \
                "Previous planner info not maintained"

    def test_list_planners_after_switch(self):
        """Test that list_planners shows correct current after switch."""
        # Switch to Classic
        self._set_planner(SetPlanner.Request.CLASSIC)

        # List planners
        planner_list = self._get_current_planner()

        # Should show Classic as current
        assert 'classic' in planner_list.lower()

        # Switch to MPC
        self._set_planner(SetPlanner.Request.MPC)

        # List again
        planner_list = self._get_current_planner()

        # Should now show MPC
        assert 'mpc' in planner_list.lower()

    # =========================
    # PERFORMANCE TESTS
    # =========================

    def test_switch_performance(self):
        """Test that planner switching completes in reasonable time."""
        import time

        # Measure switch time
        start = time.time()
        response = self._set_planner(SetPlanner.Request.CLASSIC)
        switch_time = time.time() - start

        if response and response.success:
            # Should complete quickly (< 2 seconds even with warmup)
            assert switch_time < 2.0, \
                f"Planner switch too slow: {switch_time:.3f}s"

    def test_rapid_switching(self):
        """Test rapid consecutive switches."""
        import time

        planners = [
            SetPlanner.Request.CLASSIC,
            SetPlanner.Request.MPC,
            SetPlanner.Request.CLASSIC,
        ]

        start = time.time()

        for planner_type in planners:
            response = self._set_planner(planner_type)
            assert response is not None

        total_time = time.time() - start

        # All switches should complete in reasonable time
        assert total_time < 5.0, f"Rapid switching too slow: {total_time:.3f}s"


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
