#!/usr/bin/env python3
"""
Integration tests for curobo_ik node.

Tests the Inverse Kinematics services that convert end-effector poses
to joint configurations with collision avoidance.

Node: curobo_ik
Services tested:
- /curobo_ik/ik_pose (curobo_msgs/srv/Ik)
- /curobo_ik/ik_batch_poses (curobo_msgs/srv/IkBatch)
- /curobo_ik/add_object (curobo_msgs/srv/AddObject)
- /curobo_ik/remove_object (curobo_msgs/srv/RemoveObject)
- /curobo_ik/remove_all_objects (std_srvs/srv/Trigger)
- /curobo_ik/get_voxel_grid (curobo_msgs/srv/GetVoxelGrid)
- /curobo_ik/get_collision_distance (curobo_msgs/srv/GetCollisionDistance)

Usage:
    pytest tests/integration/test_ik_services.py
"""

import pytest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger
from curobo_msgs.srv import (
    Ik,
    IkBatch,
    AddObject,
    RemoveObject,
    GetVoxelGrid,
    GetCollisionDistance
)

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from fixtures.test_poses import TestPoses, TestJointStates
from fixtures.test_robot_configs import TestRobotConfig


class TestIKServices:
    """Test suite for IK services."""

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
        self.node = Node('test_ik_node')

        # Create service clients
        self.ik_client = self.node.create_client(Ik, '/curobo_ik/ik_pose')
        self.ik_batch_client = self.node.create_client(IkBatch, '/curobo_ik/ik_batch_poses')
        self.add_object_client = self.node.create_client(AddObject, '/curobo_ik/add_object')
        self.remove_object_client = self.node.create_client(RemoveObject, '/curobo_ik/remove_object')
        self.remove_all_client = self.node.create_client(Trigger, '/curobo_ik/remove_all_objects')
        self.voxel_grid_client = self.node.create_client(GetVoxelGrid, '/curobo_ik/get_voxel_grid')
        self.collision_dist_client = self.node.create_client(
            GetCollisionDistance,
            '/curobo_ik/get_collision_distance'
        )

    def teardown_method(self):
        """Clean up obstacles and destroy node after each test."""
        # Remove all objects before destroying node
        if self.remove_all_client.wait_for_service(timeout_sec=1.0):
            request = Trigger.Request()
            future = self.remove_all_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)

        self.node.destroy_node()

    # =========================
    # SERVICE AVAILABILITY TESTS
    # =========================

    def test_all_services_available(self):
        """Test that all IK services are available."""
        services = [
            (self.ik_client, '/curobo_ik/ik_pose'),
            (self.ik_batch_client, '/curobo_ik/ik_batch_poses'),
            (self.add_object_client, '/curobo_ik/add_object'),
            (self.remove_object_client, '/curobo_ik/remove_object'),
            (self.remove_all_client, '/curobo_ik/remove_all_objects'),
            (self.voxel_grid_client, '/curobo_ik/get_voxel_grid'),
            (self.collision_dist_client, '/curobo_ik/get_collision_distance'),
        ]

        for client, service_name in services:
            assert client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT), \
                f"Service {service_name} not available"

    # =========================
    # IK SINGLE POSE TESTS
    # =========================

    def test_ik_reachable_pose(self):
        """Test IK for a reachable pose."""
        assert self.ik_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Create request
        request = Ik.Request()
        request.target_pose = TestPoses.reach_pose_1()

        # Call service
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Verify response
        assert future.result() is not None, "IK service call failed"
        response = future.result()
        assert response.success, f"IK failed: {response.message}"
        assert response.joint_state is not None, "Joint state is None"
        assert len(response.joint_state.position) == TestRobotConfig.NUM_JOINTS, \
            f"Expected {TestRobotConfig.NUM_JOINTS} joints"

    def test_ik_unreachable_pose(self):
        """Test IK for an unreachable pose."""
        assert self.ik_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Create request with unreachable pose
        request = Ik.Request()
        request.target_pose = TestPoses.unreachable_pose()

        # Call service
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Verify response
        assert future.result() is not None, "IK service call failed"
        response = future.result()
        assert not response.success, "IK should fail for unreachable pose"

    def test_ik_multiple_solutions(self):
        """Test IK returns valid solutions for different reachable poses."""
        assert self.ik_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        test_poses = [
            TestPoses.reach_pose_1(),
            TestPoses.reach_pose_2(),
            TestPoses.reach_pose_3(),
        ]

        for i, pose in enumerate(test_poses):
            request = Ik.Request()
            request.target_pose = pose

            future = self.ik_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

            response = future.result()
            assert response is not None, f"IK call {i} failed"
            if response.success:
                assert len(response.joint_state.position) == TestRobotConfig.NUM_JOINTS

    # =========================
    # IK BATCH TESTS
    # =========================

    def test_ik_batch_multiple_poses(self):
        """Test batch IK for multiple poses."""
        assert self.ik_batch_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Create batch request
        request = IkBatch.Request()
        request.target_poses = [
            TestPoses.reach_pose_1(),
            TestPoses.reach_pose_2(),
            TestPoses.reach_pose_3(),
        ]

        # Call service
        future = self.ik_batch_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Verify response
        assert future.result() is not None, "IK batch service call failed"
        response = future.result()
        assert len(response.joint_states) == len(request.target_poses), \
            "Batch size mismatch"
        assert len(response.success_flags) == len(request.target_poses), \
            "Success flags size mismatch"

    def test_ik_batch_large(self):
        """Test batch IK with large batch (100 poses)."""
        assert self.ik_batch_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Create large batch with variations of reachable poses
        batch_size = 100
        request = IkBatch.Request()
        base_pose = TestPoses.reach_pose_1()

        for i in range(batch_size):
            pose = Pose()
            pose.position.x = base_pose.position.x + (i % 10) * 0.01
            pose.position.y = base_pose.position.y
            pose.position.z = base_pose.position.z
            pose.orientation = base_pose.orientation
            request.target_poses.append(pose)

        # Call service (allow more time for large batch)
        future = self.ik_batch_client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 3
        )

        # Verify response
        assert future.result() is not None, "IK large batch failed"
        response = future.result()
        assert len(response.joint_states) == batch_size
        assert len(response.success_flags) == batch_size

    def test_ik_batch_empty(self):
        """Test batch IK with empty request."""
        assert self.ik_batch_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Create empty request
        request = IkBatch.Request()
        request.target_poses = []

        # Call service
        future = self.ik_batch_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Should handle gracefully
        assert future.result() is not None
        response = future.result()
        assert len(response.joint_states) == 0
        assert len(response.success_flags) == 0

    # =========================
    # OBSTACLE MANAGEMENT TESTS
    # =========================

    def test_add_cuboid_obstacle(self):
        """Test adding a cuboid obstacle."""
        assert self.add_object_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Create request
        request = AddObject.Request()
        request.object_name = "test_cuboid"
        request.object_type = "cuboid"
        pose_dict = TestRobotConfig.get_test_object_pose()
        request.pose.position.x = pose_dict['position']['x']
        request.pose.position.y = pose_dict['position']['y']
        request.pose.position.z = pose_dict['position']['z']
        request.pose.orientation.w = pose_dict['orientation']['w']
        request.dimensions = TestRobotConfig.TEST_CUBOID_DIMS

        # Call service
        future = self.add_object_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Verify response
        assert future.result() is not None, "Add object service call failed"
        response = future.result()
        assert response.success, f"Failed to add cuboid: {response.message}"

    def test_add_sphere_obstacle(self):
        """Test adding a sphere obstacle."""
        assert self.add_object_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Create request
        request = AddObject.Request()
        request.object_name = "test_sphere"
        request.object_type = "sphere"
        pose_dict = TestRobotConfig.get_test_object_pose()
        request.pose.position.x = pose_dict['position']['x']
        request.pose.position.y = pose_dict['position']['y']
        request.pose.position.z = pose_dict['position']['z']
        request.dimensions = [TestRobotConfig.TEST_SPHERE_RADIUS]

        # Call service
        future = self.add_object_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Verify response
        assert future.result() is not None
        response = future.result()
        assert response.success, f"Failed to add sphere: {response.message}"

    def test_remove_specific_obstacle(self):
        """Test removing a specific obstacle."""
        # First add an obstacle
        assert self.add_object_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)
        add_req = AddObject.Request()
        add_req.object_name = "test_remove"
        add_req.object_type = "cuboid"
        pose_dict = TestRobotConfig.get_test_object_pose()
        add_req.pose.position.x = pose_dict['position']['x']
        add_req.pose.position.y = pose_dict['position']['y']
        add_req.pose.position.z = pose_dict['position']['z']
        add_req.pose.orientation.w = pose_dict['orientation']['w']
        add_req.dimensions = TestRobotConfig.TEST_CUBOID_DIMS

        future = self.add_object_client.call_async(add_req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)
        assert future.result().success

        # Now remove it
        assert self.remove_object_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)
        remove_req = RemoveObject.Request()
        remove_req.object_name = "test_remove"

        future = self.remove_object_client.call_async(remove_req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Verify response
        assert future.result() is not None
        response = future.result()
        assert response.success, f"Failed to remove object: {response.message}"

    def test_remove_nonexistent_obstacle(self):
        """Test removing a non-existent obstacle."""
        assert self.remove_object_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        request = RemoveObject.Request()
        request.object_name = "nonexistent_object"

        future = self.remove_object_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Should handle gracefully (may succeed or fail depending on implementation)
        assert future.result() is not None

    def test_remove_all_obstacles(self):
        """Test removing all obstacles."""
        # Add multiple obstacles
        assert self.add_object_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        for i in range(3):
            add_req = AddObject.Request()
            add_req.object_name = f"test_obj_{i}"
            add_req.object_type = "cuboid"
            pose_dict = TestRobotConfig.get_test_object_pose()
            add_req.pose.position.x = pose_dict['position']['x'] + i * 0.1
            add_req.pose.position.y = pose_dict['position']['y']
            add_req.pose.position.z = pose_dict['position']['z']
            add_req.pose.orientation.w = pose_dict['orientation']['w']
            add_req.dimensions = TestRobotConfig.TEST_CUBOID_DIMS

            future = self.add_object_client.call_async(add_req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Remove all
        assert self.remove_all_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)
        request = Trigger.Request()

        future = self.remove_all_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Verify response
        assert future.result() is not None
        response = future.result()
        assert response.success, "Failed to remove all objects"

    # =========================
    # COLLISION QUERY TESTS
    # =========================

    def test_get_voxel_grid_empty(self):
        """Test getting voxel grid with no obstacles."""
        # Remove all obstacles first
        assert self.remove_all_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)
        remove_req = Trigger.Request()
        future = self.remove_all_client.call_async(remove_req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Get voxel grid
        assert self.voxel_grid_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)
        request = GetVoxelGrid.Request()

        future = self.voxel_grid_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Verify response
        assert future.result() is not None
        response = future.result()
        assert response.voxel_grid is not None

    def test_get_voxel_grid_with_obstacles(self):
        """Test getting voxel grid with obstacles."""
        # Add obstacle
        assert self.add_object_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)
        add_req = AddObject.Request()
        add_req.object_name = "voxel_test"
        add_req.object_type = "cuboid"
        pose_dict = TestRobotConfig.get_test_object_pose()
        add_req.pose.position.x = pose_dict['position']['x']
        add_req.pose.position.y = pose_dict['position']['y']
        add_req.pose.position.z = pose_dict['position']['z']
        add_req.pose.orientation.w = pose_dict['orientation']['w']
        add_req.dimensions = TestRobotConfig.TEST_CUBOID_DIMS

        future = self.add_object_client.call_async(add_req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Get voxel grid
        assert self.voxel_grid_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)
        request = GetVoxelGrid.Request()

        future = self.voxel_grid_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Verify response
        assert future.result() is not None
        response = future.result()
        assert response.voxel_grid is not None

    def test_get_collision_distance(self):
        """Test getting collision distance for a joint state."""
        assert self.collision_dist_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Create request
        request = GetCollisionDistance.Request()
        request.joint_state = TestJointStates.home_state()

        # Call service
        future = self.collision_dist_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Verify response
        assert future.result() is not None
        response = future.result()
        # Distance should be a valid float (could be positive for free space)
        assert response.distance is not None

    # =========================
    # IK WITH COLLISION TESTS
    # =========================

    def test_ik_with_collision_avoidance(self):
        """Test IK avoids collisions with obstacles."""
        # Add obstacle near target pose
        assert self.add_object_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)
        add_req = AddObject.Request()
        add_req.object_name = "collision_test"
        add_req.object_type = "cuboid"
        target_pose = TestPoses.reach_pose_1()
        add_req.pose.position.x = target_pose.position.x + 0.15
        add_req.pose.position.y = target_pose.position.y
        add_req.pose.position.z = target_pose.position.z
        add_req.pose.orientation.w = 1.0
        add_req.dimensions = TestRobotConfig.TEST_CUBOID_DIMS

        future = self.add_object_client.call_async(add_req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)
        assert future.result().success

        # Try IK - should find collision-free solution or fail
        assert self.ik_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)
        ik_req = Ik.Request()
        ik_req.target_pose = target_pose

        future = self.ik_client.call_async(ik_req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Verify response exists (success depends on whether collision-free path exists)
        assert future.result() is not None


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
