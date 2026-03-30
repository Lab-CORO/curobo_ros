
"""
Auto-generated test file
Generated from YAML specification
DO NOT EDIT - Changes will be overwritten
"""

import unittest
import rclpy
from rclpy.node import Node
from rosidl_runtime_py import set_message_fields
import launch
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node as LaunchNode
import launch_testing
import launch_testing.actions
import time
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from curobo_msgs.srv import AddObject
from std_srvs.srv import Trigger
from curobo_msgs.srv import GetVoxelGrid
from curobo_msgs.srv import RemoveObject

def generate_test_description():
    """Generate launch description for test."""
    
    launch_file_0_path = PathJoinSubstitution([
        FindPackageShare('curobo_ros'),
        'launch',
        'gen_traj.launch.py'
    ])

    launch_file_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_0_path)
    )
    
    return LaunchDescription([
        launch_file_0,
        launch_testing.util.KeepAliveProc(),
        # Wait 10.0s for nodes to stabilize
        TimerAction(
            period=10.0,
            actions=[launch_testing.actions.ReadyToTest()]
        ),
    ]), locals()



class GeneratedTestSuite(unittest.TestCase):
    """Generated test class"""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_node')

    def tearDown(self):
        self.node.destroy_node()


    def test_01_add_object(self):
        """01 Add object"""

        # Create service client
        client = self.node.create_client(AddObject, '/unified_planner/add_object')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/add_object' not available after {timeout}s")

        # Create request
        request = AddObject.Request()
        set_message_fields(request, {'type': 0, 'name': 'cube', 'mesh_file_path': '', 'pose': {'position': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}}, 'dimensions': {'x': 0.5, 'y': 0.5, 'z': 0.5}, 'color': {'r': 0.0, 'a': 0.0}})

        # Call service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)

        # Check if call completed
        if not future.done():
            self.fail("Service call to '/unified_planner/add_object' timed out")

        # Get response
        response = future.result()
        if response is None:
            self.fail("Service call to '/unified_planner/add_object' failed")


        self.assertEqual(
            response.success,
            True,
            f"Field 'success' doesn't match expected value"
        )

        self.assertEqual(
            response.message,
            "Object 'cube' added successfully (1 cuboids, 0 mesh in world)",
            f"Field 'message' doesn't match expected value"
        )

    def test_02_add_second_object(self):
        """02 Add second object"""

        # Create service client
        client = self.node.create_client(AddObject, '/unified_planner/add_object')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/add_object' not available after {timeout}s")

        # Create request
        request = AddObject.Request()
        set_message_fields(request, {'type': 0, 'name': 'cube2', 'mesh_file_path': '', 'pose': {'position': {'x': 0.5, 'y': 0.5, 'z': 0.5}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}}, 'dimensions': {'x': 0.5, 'y': 0.5, 'z': 0.5}, 'color': {'r': 0.0, 'a': 0.0}})

        # Call service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)

        # Check if call completed
        if not future.done():
            self.fail("Service call to '/unified_planner/add_object' timed out")

        # Get response
        response = future.result()
        if response is None:
            self.fail("Service call to '/unified_planner/add_object' failed")


        self.assertEqual(
            response.success,
            True,
            f"Field 'success' doesn't match expected value"
        )

        self.assertEqual(
            response.message,
            "Object 'cube2' added successfully (2 cuboids, 0 mesh in world)",
            f"Field 'message' doesn't match expected value"
        )

    def test_03_get_obstacles(self):
        """03 Get obstacles"""

        # Create service client
        client = self.node.create_client(Trigger, '/unified_planner/get_obstacles')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/get_obstacles' not available after {timeout}s")

        # Create request
        request = Trigger.Request()
        set_message_fields(request, {})

        # Call service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)

        # Check if call completed
        if not future.done():
            self.fail("Service call to '/unified_planner/get_obstacles' timed out")

        # Get response
        response = future.result()
        if response is None:
            self.fail("Service call to '/unified_planner/get_obstacles' failed")


        self.assertEqual(
            response.success,
            True,
            f"Field 'success' doesn't match expected value"
        )

    def test_04_get_voxel_grid(self):
        """04 Get voxel grid"""

        # Create service client
        client = self.node.create_client(GetVoxelGrid, '/unified_planner/get_voxel_grid')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/get_voxel_grid' not available after {timeout}s")

        # Create request
        request = GetVoxelGrid.Request()
        set_message_fields(request, {})

        # Call service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)

        # Check if call completed
        if not future.done():
            self.fail("Service call to '/unified_planner/get_voxel_grid' timed out")

        # Get response
        response = future.result()
        if response is None:
            self.fail("Service call to '/unified_planner/get_voxel_grid' failed")


        self.assertEqual(
            response.voxel_grid.size_x,
            61,
            f"Field 'size_x' doesn't match expected value"
        )

        self.assertEqual(
            response.voxel_grid.size_y,
            61,
            f"Field 'size_y' doesn't match expected value"
        )

        self.assertEqual(
            response.voxel_grid.size_z,
            61,
            f"Field 'size_z' doesn't match expected value"
        )

        self.assertEqual(
            len(list(response.voxel_grid.data)),
            226981,
            f"Field 'data' length doesn't match expected 226981"
        )

    def test_05_remove_object(self):
        """05 Remove object"""

        # Create service client
        client = self.node.create_client(RemoveObject, '/unified_planner/remove_object')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/remove_object' not available after {timeout}s")

        # Create request
        request = RemoveObject.Request()
        set_message_fields(request, {'name': 'cube'})

        # Call service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)

        # Check if call completed
        if not future.done():
            self.fail("Service call to '/unified_planner/remove_object' timed out")

        # Get response
        response = future.result()
        if response is None:
            self.fail("Service call to '/unified_planner/remove_object' failed")


        self.assertEqual(
            response.success,
            True,
            f"Field 'success' doesn't match expected value"
        )

        self.assertEqual(
            response.message,
            "Object 'cube' removed successfully (cuboid)",
            f"Field 'message' doesn't match expected value"
        )

    def test_06_remove_all_object(self):
        """06 Remove all object"""

        # Create service client
        client = self.node.create_client(Trigger, '/unified_planner/remove_all_objects')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/remove_all_objects' not available after {timeout}s")

        # Create request
        request = Trigger.Request()
        set_message_fields(request, {})

        # Call service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)

        # Check if call completed
        if not future.done():
            self.fail("Service call to '/unified_planner/remove_all_objects' timed out")

        # Get response
        response = future.result()
        if response is None:
            self.fail("Service call to '/unified_planner/remove_all_objects' failed")


        self.assertEqual(
            response.success,
            True,
            f"Field 'success' doesn't match expected value"
        )

        self.assertEqual(
            response.message,
            'All objects removed successfully (1 cuboids, 0 meshes)',
            f"Field 'message' doesn't match expected value"
        )

@launch_testing.post_shutdown_test()
class PostShutdownTests(unittest.TestCase):
    """Post-shutdown tests to validate clean exit"""

    def test_exit_codes(self, proc_info):
        """Test that all processes exited without critical errors"""
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, 1, -2, -6, -11]  # 0: clean, 1: shutdown error, -2: SIGINT, -6: rviz2 SIGABRT, -11: rviz2 SIGSEGV on shutdown
        )

