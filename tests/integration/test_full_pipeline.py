#!/usr/bin/env python3
"""
End-to-end integration tests for full curobo_ros pipeline.

Tests complete workflows involving multiple nodes and services:
- IK → Trajectory Generation → Execution
- Obstacle management → Planning with avoidance
- Multi-node coordination

Usage:
    pytest tests/integration/test_full_pipeline.py
"""

import pytest
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from curobo_msgs.srv import (
    Ik,
    Fk,
    TrajectoryGeneration,
    AddObject,
    RemoveObject,
    SetPlanner
)
from curobo_msgs.action import SendTrajectory

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from fixtures.test_poses import TestPoses, TestJointStates
from fixtures.test_robot_configs import TestRobotConfig


class TestFullPipeline:
    """Test suite for end-to-end integration."""

    @classmethod
    def setup_class(cls):
        """Initialize ROS2 for all tests."""
        rclpy.init()

    @classmethod
    def teardown_class(cls):
        """Shutdown ROS2 after all tests."""
        rclpy.shutdown()

    def setup_method(self):
        """Create test node and all service clients before each test."""
        self.node = Node('test_full_pipeline_node')

        # FK/IK clients
        self.fk_client = self.node.create_client(Fk, '/curobo/fk_poses')
        self.ik_client = self.node.create_client(Ik, '/curobo_ik/ik_pose')

        # Trajectory generation clients (try both old and new nodes)
        self.gen_traj_client_old = self.node.create_client(
            TrajectoryGeneration,
            '/curobo_gen_traj/generate_trajectory'
        )
        self.gen_traj_client_new = self.node.create_client(
            TrajectoryGeneration,
            '/unified_planner/generate_trajectory'
        )

        # Obstacle management
        self.add_object_client = self.node.create_client(
            AddObject,
            '/curobo_gen_traj/add_object'
        )
        self.remove_all_client = self.node.create_client(
            Trigger,
            '/curobo_gen_traj/remove_all_objects'
        )

        # Planner switching (new node only)
        self.set_planner_client = self.node.create_client(
            SetPlanner,
            '/unified_planner/set_planner'
        )

        # Action clients
        self.execute_action_old = ActionClient(
            self.node,
            SendTrajectory,
            '/curobo_gen_traj/send_trajectory'
        )
        self.execute_action_new = ActionClient(
            self.node,
            SendTrajectory,
            '/unified_planner/execute_trajectory'
        )

    def teardown_method(self):
        """Clean up obstacles and destroy node after each test."""
        # Remove all objects
        if self.remove_all_client.wait_for_service(timeout_sec=1.0):
            request = Trigger.Request()
            future = self.remove_all_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)

        self.node.destroy_node()

    # =========================
    # FK → IK PIPELINE TESTS
    # =========================

    def test_fk_then_ik_roundtrip(self):
        """Test FK → IK roundtrip: joint state → pose → joint state."""
        # Step 1: FK (joint state → pose)
        assert self.fk_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        fk_req = Fk.Request()
        fk_req.joint_states = [TestJointStates.valid_state_1()]

        fk_future = self.fk_client.call_async(fk_req)
        rclpy.spin_until_future_complete(self.node, fk_future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        fk_response = fk_future.result()
        assert fk_response is not None, "FK failed"
        assert len(fk_response.poses) == 1
        target_pose = fk_response.poses[0]

        # Step 2: IK (pose → joint state)
        assert self.ik_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        ik_req = Ik.Request()
        ik_req.target_pose = target_pose

        ik_future = self.ik_client.call_async(ik_req)
        rclpy.spin_until_future_complete(self.node, ik_future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        ik_response = ik_future.result()
        assert ik_response is not None, "IK failed"
        assert ik_response.success, "IK should find solution for FK pose"

    # =========================
    # IK → TRAJECTORY GENERATION PIPELINE
    # =========================

    def test_ik_then_trajectory_generation(self):
        """Test IK → Trajectory Generation pipeline."""
        # Step 1: IK to find valid joint configuration
        assert self.ik_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        ik_req = Ik.Request()
        ik_req.target_pose = TestPoses.reach_pose_1()

        ik_future = self.ik_client.call_async(ik_req)
        rclpy.spin_until_future_complete(self.node, ik_future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        ik_response = ik_future.result()

        if ik_response and ik_response.success:
            # Step 2: Generate trajectory to same pose
            # Try new node first, fallback to old
            if self.gen_traj_client_new.wait_for_service(timeout_sec=1.0):
                gen_client = self.gen_traj_client_new
            elif self.gen_traj_client_old.wait_for_service(timeout_sec=1.0):
                gen_client = self.gen_traj_client_old
            else:
                pytest.skip("No trajectory generation node available")

            traj_req = TrajectoryGeneration.Request()
            traj_req.target_pose = TestPoses.reach_pose_1()

            traj_future = gen_client.call_async(traj_req)
            rclpy.spin_until_future_complete(
                self.node,
                traj_future,
                timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
            )

            traj_response = traj_future.result()
            assert traj_response is not None, "Trajectory generation failed"

    # =========================
    # OBSTACLE → PLANNING PIPELINE
    # =========================

    def test_add_obstacle_then_plan(self):
        """Test adding obstacle and planning collision-free trajectory."""
        # Step 1: Add obstacle
        assert self.add_object_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        add_req = AddObject.Request()
        add_req.object_name = "test_obstacle"
        add_req.object_type = "cuboid"
        add_req.pose.position.x = 0.4
        add_req.pose.position.y = 0.0
        add_req.pose.position.z = 0.3
        add_req.pose.orientation.w = 1.0
        add_req.dimensions = TestRobotConfig.TEST_CUBOID_DIMS

        add_future = self.add_object_client.call_async(add_req)
        rclpy.spin_until_future_complete(self.node, add_future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        add_response = add_future.result()
        assert add_response is not None
        assert add_response.success, "Failed to add obstacle"

        # Step 2: Plan trajectory (should avoid obstacle)
        if self.gen_traj_client_new.wait_for_service(timeout_sec=1.0):
            gen_client = self.gen_traj_client_new
        elif self.gen_traj_client_old.wait_for_service(timeout_sec=1.0):
            gen_client = self.gen_traj_client_old
        else:
            pytest.skip("No trajectory generation node available")

        traj_req = TrajectoryGeneration.Request()
        traj_req.target_pose = TestPoses.reach_pose_1()

        traj_future = gen_client.call_async(traj_req)
        rclpy.spin_until_future_complete(
            self.node,
            traj_future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
        )

        traj_response = traj_future.result()
        assert traj_response is not None
        # Trajectory should avoid obstacle (success) or fail if no path

    # =========================
    # FULL EXECUTION PIPELINE
    # =========================

    def test_plan_and_execute_pipeline(self):
        """Test complete pipeline: plan → execute."""
        # Try new node first
        if self.gen_traj_client_new.wait_for_service(timeout_sec=1.0):
            gen_client = self.gen_traj_client_new
            exec_client = self.execute_action_new
        elif self.gen_traj_client_old.wait_for_service(timeout_sec=1.0):
            gen_client = self.gen_traj_client_old
            exec_client = self.execute_action_old
        else:
            pytest.skip("No trajectory generation node available")

        # Step 1: Plan
        traj_req = TrajectoryGeneration.Request()
        traj_req.target_pose = TestPoses.reach_pose_1()

        traj_future = gen_client.call_async(traj_req)
        rclpy.spin_until_future_complete(
            self.node,
            traj_future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
        )

        traj_response = traj_future.result()
        assert traj_response is not None

        if traj_response.success:
            # Step 2: Execute
            assert exec_client.wait_for_server(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

            goal_msg = SendTrajectory.Goal()
            send_goal_future = exec_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(
                self.node,
                send_goal_future,
                timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
            )

            goal_handle = send_goal_future.result()
            assert goal_handle is not None

    # =========================
    # MULTI-PLANNER PIPELINE (NEW NODE)
    # =========================

    def test_classic_then_mpc_pipeline(self):
        """Test planning with Classic then MPC planner."""
        if not self.set_planner_client.wait_for_service(timeout_sec=1.0):
            pytest.skip("Unified planner not available")

        # Plan with Classic
        set_req = SetPlanner.Request()
        set_req.planner_type = SetPlanner.Request.CLASSIC

        set_future = self.set_planner_client.call_async(set_req)
        rclpy.spin_until_future_complete(self.node, set_future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        traj_req = TrajectoryGeneration.Request()
        traj_req.target_pose = TestPoses.reach_pose_1()

        gen_future = self.gen_traj_client_new.call_async(traj_req)
        rclpy.spin_until_future_complete(
            self.node,
            gen_future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
        )

        classic_result = gen_future.result()

        # Plan with MPC
        set_req.planner_type = SetPlanner.Request.MPC
        set_future = self.set_planner_client.call_async(set_req)
        rclpy.spin_until_future_complete(
            self.node,
            set_future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
        )

        traj_req.target_pose = TestPoses.reach_pose_2()
        gen_future = self.gen_traj_client_new.call_async(traj_req)
        rclpy.spin_until_future_complete(
            self.node,
            gen_future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
        )

        mpc_result = gen_future.result()

        # Both should complete
        assert classic_result is not None
        assert mpc_result is not None

    # =========================
    # MULTI-SERVICE COORDINATION
    # =========================

    def test_ik_obstacle_trajectory_pipeline(self):
        """Test IK → Add Obstacle → Trajectory Generation pipeline."""
        # Step 1: IK
        assert self.ik_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        ik_req = Ik.Request()
        ik_req.target_pose = TestPoses.reach_pose_2()

        ik_future = self.ik_client.call_async(ik_req)
        rclpy.spin_until_future_complete(self.node, ik_future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Step 2: Add obstacle
        assert self.add_object_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        add_req = AddObject.Request()
        add_req.object_name = "pipeline_obstacle"
        add_req.object_type = "sphere"
        add_req.pose.position.x = 0.5
        add_req.pose.position.y = 0.2
        add_req.pose.position.z = 0.4
        add_req.dimensions = [TestRobotConfig.TEST_SPHERE_RADIUS]

        add_future = self.add_object_client.call_async(add_req)
        rclpy.spin_until_future_complete(self.node, add_future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Step 3: Plan trajectory
        if self.gen_traj_client_new.wait_for_service(timeout_sec=1.0):
            gen_client = self.gen_traj_client_new
        elif self.gen_traj_client_old.wait_for_service(timeout_sec=1.0):
            gen_client = self.gen_traj_client_old
        else:
            pytest.skip("No trajectory generation node available")

        traj_req = TrajectoryGeneration.Request()
        traj_req.target_pose = TestPoses.reach_pose_2()

        traj_future = gen_client.call_async(traj_req)
        rclpy.spin_until_future_complete(
            self.node,
            traj_future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
        )

        # All steps should complete
        assert traj_future.result() is not None

    # =========================
    # SEQUENTIAL TRAJECTORIES
    # =========================

    def test_sequential_trajectory_execution(self):
        """Test executing multiple trajectories sequentially."""
        if self.gen_traj_client_new.wait_for_service(timeout_sec=1.0):
            gen_client = self.gen_traj_client_new
            exec_client = self.execute_action_new
        elif self.gen_traj_client_old.wait_for_service(timeout_sec=1.0):
            gen_client = self.gen_traj_client_old
            exec_client = self.execute_action_old
        else:
            pytest.skip("No trajectory generation node available")

        poses = [
            TestPoses.reach_pose_1(),
            TestPoses.reach_pose_2(),
        ]

        for pose in poses:
            # Plan
            traj_req = TrajectoryGeneration.Request()
            traj_req.target_pose = pose

            traj_future = gen_client.call_async(traj_req)
            rclpy.spin_until_future_complete(
                self.node,
                traj_future,
                timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
            )

            if traj_future.result() and traj_future.result().success:
                # Execute (cancel quickly for test speed)
                assert exec_client.wait_for_server(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

                goal_msg = SendTrajectory.Goal()
                send_goal_future = exec_client.send_goal_async(goal_msg)
                rclpy.spin_until_future_complete(
                    self.node,
                    send_goal_future,
                    timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
                )

                goal_handle = send_goal_future.result()
                if goal_handle and goal_handle.accepted and goal_handle.is_active:
                    # Cancel to move to next quickly
                    cancel_future = goal_handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(
                        self.node,
                        cancel_future,
                        timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
                    )


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
