
"""
Auto-generated test file
Generated from YAML specification
DO NOT EDIT - Changes will be overwritten
"""

import unittest
import rclpy
from rclpy.node import Node
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


    def test_add_object(self):
        """Add object"""

        # Create service client
        client = self.node.create_client(AddObject, '/unified_planner/add_object')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/add_object' not available after {timeout}s")

        # Create request
        request = AddObject.Request()
        request.type = 0
        request.name = 'cube'
        request.mesh_file_path = ''
        request.dimensions.x = 0.5
        request.dimensions.y = 0.5
        request.dimensions.z = 0.5
        request.color.r = 0.0
        request.color.a = 0.0
        request.pose.position.x = 0.0
        request.pose.position.y = 0.0
        request.pose.position.z = 0.0
        request.pose.orientation.x = 0.0
        request.pose.orientation.y = 0.0
        request.pose.orientation.z = 0.0
        request.pose.orientation.w = 0.0

        # Call service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)

        # Check if call completed
        if not future.done():
            self.fail(f"Service call to '{service_name}' timed out")

        # Get response
        response = future.result()
        if response is None:
            self.fail(f"Service call to '{service_name}' failed")


        self.assertEqual(
            response.success,
            True,
            f"Field 'success' doesn't match expected value"
        )

        self.assertEqual(
            response.message,
            'Object cube added successfully',
            f"Field 'message' doesn't match expected value"
        )

    def test_remove_object(self):
        """Remove object"""

        # Create service client
        client = self.node.create_client(RemoveObject, '/unified_planner/remove_object')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/remove_object' not available after {timeout}s")

        # Create request
        request = RemoveObject.Request()
        request.name = 'cube'

        # Call service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)

        # Check if call completed
        if not future.done():
            self.fail(f"Service call to '{service_name}' timed out")

        # Get response
        response = future.result()
        if response is None:
            self.fail(f"Service call to '{service_name}' failed")


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

@launch_testing.post_shutdown_test()
class PostShutdownTests(unittest.TestCase):
    """Post-shutdown tests to validate clean exit"""

    def test_exit_codes(self, proc_info):
        """Test that all processes exited without critical errors"""
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, 1, -2, -11]  # 0: clean, 1: shutdown error, -2: SIGINT, -11: rviz2 SIGSEGV on shutdown
        )

