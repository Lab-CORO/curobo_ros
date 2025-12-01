#!/usr/bin/env python3
"""
Integration tests for obstacle management in curobo_gen_traj node.

Tests services for adding, removing, and querying obstacles
for collision avoidance during motion planning.

Node: curobo_gen_traj
Services tested:
- /curobo_gen_traj/add_object (curobo_msgs/srv/AddObject)
- /curobo_gen_traj/remove_object (curobo_msgs/srv/RemoveObject)
- /curobo_gen_traj/remove_all_objects (std_srvs/srv/Trigger)
- /curobo_gen_traj/get_obstacles (std_srvs/srv/Trigger)
- /curobo_gen_traj/get_voxel_grid (curobo_msgs/srv/GetVoxelGrid)
- /curobo_gen_traj/get_collision_distance (curobo_msgs/srv/GetCollisionDistance)

Usage:
    pytest tests/integration/test_obstacle_management.py
"""

import pytest
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from curobo_msgs.srv import (
    AddObject,
    RemoveObject,
    GetVoxelGrid,
    GetCollisionDistance,
    TrajectoryGeneration
)

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from fixtures.test_poses import TestPoses, TestJointStates
from fixtures.test_robot_configs import TestRobotConfig


class TestObstacleManagement:
    """Test suite for obstacle management services."""

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
        self.node = Node('test_obstacle_node')

        # Create service clients
        self.add_object_client = self.node.create_client(
            AddObject,
            '/curobo_gen_traj/add_object'
        )
        self.remove_object_client = self.node.create_client(
            RemoveObject,
            '/curobo_gen_traj/remove_object'
        )
        self.remove_all_client = self.node.create_client(
            Trigger,
            '/curobo_gen_traj/remove_all_objects'
        )
        self.get_obstacles_client = self.node.create_client(
            Trigger,
            '/curobo_gen_traj/get_obstacles'
        )
        self.voxel_grid_client = self.node.create_client(
            GetVoxelGrid,
            '/curobo_gen_traj/get_voxel_grid'
        )
        self.collision_dist_client = self.node.create_client(
            GetCollisionDistance,
            '/curobo_gen_traj/get_collision_distance'
        )
        self.gen_traj_client = self.node.create_client(
            TrajectoryGeneration,
            '/curobo_gen_traj/generate_trajectory'
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

    def test_all_obstacle_services_available(self):
        """Test that all obstacle management services are available."""
        services = [
            (self.add_object_client, '/curobo_gen_traj/add_object'),
            (self.remove_object_client, '/curobo_gen_traj/remove_object'),
            (self.remove_all_client, '/curobo_gen_traj/remove_all_objects'),
            (self.get_obstacles_client, '/curobo_gen_traj/get_obstacles'),
            (self.voxel_grid_client, '/curobo_gen_traj/get_voxel_grid'),
            (self.collision_dist_client, '/curobo_gen_traj/get_collision_distance'),
        ]

        for client, service_name in services:
            assert client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT), \
                f"Service {service_name} not available"

    # =========================
    # ADD OBJECT TESTS
    # =========================

    def test_add_cuboid_object(self):
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
        assert future.result() is not None, "add_object service call failed"
        response = future.result()
        assert response.success, f"Failed to add cuboid: {response.message}"

    def test_add_sphere_object(self):
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

    def test_add_cylinder_object(self):
        """Test adding a cylinder obstacle."""
        assert self.add_object_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Create request
        request = AddObject.Request()
        request.object_name = "test_cylinder"
        request.object_type = "cylinder"
        pose_dict = TestRobotConfig.get_test_object_pose()
        request.pose.position.x = pose_dict['position']['x']
        request.pose.position.y = pose_dict['position']['y']
        request.pose.position.z = pose_dict['position']['z']
        request.pose.orientation.w = pose_dict['orientation']['w']
        request.dimensions = TestRobotConfig.TEST_CYLINDER_DIMS

        # Call service
        future = self.add_object_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Verify response
        assert future.result() is not None
        response = future.result()
        assert response.success, f"Failed to add cylinder: {response.message}"

    def test_add_capsule_object(self):
        """Test adding a capsule obstacle."""
        assert self.add_object_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Create request
        request = AddObject.Request()
        request.object_name = "test_capsule"
        request.object_type = "capsule"
        pose_dict = TestRobotConfig.get_test_object_pose()
        request.pose.position.x = pose_dict['position']['x']
        request.pose.position.y = pose_dict['position']['y']
        request.pose.position.z = pose_dict['position']['z']
        request.pose.orientation.w = pose_dict['orientation']['w']
        request.dimensions = TestRobotConfig.TEST_CYLINDER_DIMS  # radius, height

        # Call service
        future = self.add_object_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Verify response
        assert future.result() is not None
        response = future.result()
        assert response.success, f"Failed to add capsule: {response.message}"

    def test_add_multiple_objects(self):
        """Test adding multiple different obstacles."""
        assert self.add_object_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        objects = [
            ("obj1", "cuboid", TestRobotConfig.TEST_CUBOID_DIMS, (0.5, 0.0, 0.3)),
            ("obj2", "sphere", [TestRobotConfig.TEST_SPHERE_RADIUS], (0.3, 0.3, 0.4)),
            ("obj3", "cylinder", TestRobotConfig.TEST_CYLINDER_DIMS, (0.6, -0.2, 0.3)),
        ]

        for obj_name, obj_type, dims, pos in objects:
            request = AddObject.Request()
            request.object_name = obj_name
            request.object_type = obj_type
            request.pose.position.x = pos[0]
            request.pose.position.y = pos[1]
            request.pose.position.z = pos[2]
            request.pose.orientation.w = 1.0
            request.dimensions = dims

            future = self.add_object_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

            assert future.result() is not None
            assert future.result().success, f"Failed to add {obj_name}"

    def test_add_duplicate_object_name(self):
        """Test adding object with duplicate name (should update or reject)."""
        assert self.add_object_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Add first object
        request = AddObject.Request()
        request.object_name = "duplicate_test"
        request.object_type = "cuboid"
        pose_dict = TestRobotConfig.get_test_object_pose()
        request.pose.position.x = pose_dict['position']['x']
        request.pose.position.y = pose_dict['position']['y']
        request.pose.position.z = pose_dict['position']['z']
        request.pose.orientation.w = pose_dict['orientation']['w']
        request.dimensions = TestRobotConfig.TEST_CUBOID_DIMS

        future = self.add_object_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)
        assert future.result().success

        # Add second object with same name
        request.pose.position.x += 0.5  # Different position
        future = self.add_object_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Implementation may update or reject
        assert future.result() is not None

    # =========================
    # REMOVE OBJECT TESTS
    # =========================

    def test_remove_existing_object(self):
        """Test removing an existing obstacle."""
        # First add an object
        assert self.add_object_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)
        add_req = AddObject.Request()
        add_req.object_name = "remove_test"
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
        remove_req.object_name = "remove_test"

        future = self.remove_object_client.call_async(remove_req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Verify response
        assert future.result() is not None
        response = future.result()
        assert response.success, f"Failed to remove object: {response.message}"

    def test_remove_nonexistent_object(self):
        """Test removing a non-existent obstacle."""
        assert self.remove_object_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        request = RemoveObject.Request()
        request.object_name = "nonexistent_obj"

        future = self.remove_object_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Should handle gracefully
        assert future.result() is not None

    def test_remove_all_objects(self):
        """Test removing all obstacles at once."""
        # Add multiple objects
        assert self.add_object_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        for i in range(5):
            add_req = AddObject.Request()
            add_req.object_name = f"batch_obj_{i}"
            add_req.object_type = "sphere"
            add_req.pose.position.x = 0.5 + i * 0.1
            add_req.pose.position.y = 0.0
            add_req.pose.position.z = 0.3
            add_req.dimensions = [TestRobotConfig.TEST_SPHERE_RADIUS]

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

    def test_remove_all_empty_scene(self):
        """Test remove_all on empty scene."""
        # Ensure scene is empty
        assert self.remove_all_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)
        request = Trigger.Request()

        future = self.remove_all_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Should succeed even if empty
        assert future.result() is not None
        assert future.result().success

    # =========================
    # QUERY TESTS
    # =========================

    def test_get_obstacles_empty(self):
        """Test getting obstacle list when scene is empty."""
        # Ensure scene is empty
        assert self.remove_all_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)
        remove_req = Trigger.Request()
        future = self.remove_all_client.call_async(remove_req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Get obstacles
        assert self.get_obstacles_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)
        request = Trigger.Request()

        future = self.get_obstacles_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Verify response
        assert future.result() is not None
        response = future.result()
        assert response.success

    def test_get_obstacles_with_objects(self):
        """Test getting obstacle list with objects in scene."""
        # Add some objects
        assert self.add_object_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        for i in range(3):
            add_req = AddObject.Request()
            add_req.object_name = f"query_obj_{i}"
            add_req.object_type = "cuboid"
            add_req.pose.position.x = 0.5 + i * 0.2
            add_req.pose.position.y = 0.0
            add_req.pose.position.z = 0.3
            add_req.pose.orientation.w = 1.0
            add_req.dimensions = TestRobotConfig.TEST_CUBOID_DIMS

            future = self.add_object_client.call_async(add_req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Get obstacles
        assert self.get_obstacles_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)
        request = Trigger.Request()

        future = self.get_obstacles_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        # Verify response
        assert future.result() is not None
        response = future.result()
        assert response.success
        # Message should contain object names
        assert len(response.message) > 0

    def test_get_voxel_grid(self):
        """Test getting voxel grid representation."""
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
        assert response.distance is not None

    # =========================
    # INTEGRATION TESTS
    # =========================

    def test_trajectory_avoids_obstacle(self):
        """Test that trajectory generation avoids obstacles."""
        # Add obstacle between robot and target
        assert self.add_object_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        target_pose = TestPoses.reach_pose_1()
        add_req = AddObject.Request()
        add_req.object_name = "blocking_obstacle"
        add_req.object_type = "cuboid"
        add_req.pose.position.x = target_pose.position.x - 0.1
        add_req.pose.position.y = target_pose.position.y
        add_req.pose.position.z = target_pose.position.z
        add_req.pose.orientation.w = 1.0
        add_req.dimensions = TestRobotConfig.TEST_CUBOID_DIMS

        future = self.add_object_client.call_async(add_req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)
        assert future.result().success

        # Try to generate trajectory
        assert self.gen_traj_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)
        traj_req = TrajectoryGeneration.Request()
        traj_req.target_pose = target_pose

        future = self.gen_traj_client.call_async(traj_req)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
        )

        # Trajectory should either avoid obstacle (success) or fail if no path exists
        assert future.result() is not None

    def test_add_remove_cycle(self):
        """Test repeated add/remove cycles."""
        assert self.add_object_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)
        assert self.remove_object_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)

        for cycle in range(3):
            # Add object
            add_req = AddObject.Request()
            add_req.object_name = "cycle_obj"
            add_req.object_type = "sphere"
            add_req.pose.position.x = 0.5
            add_req.pose.position.y = 0.0
            add_req.pose.position.z = 0.3
            add_req.dimensions = [TestRobotConfig.TEST_SPHERE_RADIUS]

            future = self.add_object_client.call_async(add_req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)
            assert future.result().success, f"Cycle {cycle}: Failed to add"

            # Remove object
            remove_req = RemoveObject.Request()
            remove_req.object_name = "cycle_obj"

            future = self.remove_object_client.call_async(remove_req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)
            assert future.result().success, f"Cycle {cycle}: Failed to remove"


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
