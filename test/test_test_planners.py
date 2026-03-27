
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
from curobo_msgs.srv import GetPlanners
from curobo_msgs.srv import SetPlanner
from curobo_msgs.srv import TrajectoryGeneration

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


    def test_01_get_planners(self):
        """01 Get planners"""

        # Create service client
        client = self.node.create_client(GetPlanners, '/unified_planner/get_planners')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/get_planners' not available after {timeout}s")

        # Create request
        request = GetPlanners.Request()
        set_message_fields(request, {})

        # Call service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)

        # Check if call completed
        if not future.done():
            self.fail("Service call to '/unified_planner/get_planners' timed out")

        # Get response
        response = future.result()
        if response is None:
            self.fail("Service call to '/unified_planner/get_planners' failed")


        self.assertEqual(
            response.success,
            True,
            f"Field 'success' doesn't match expected value"
        )

    def test_02_set_planner_multipoint(self):
        """02 Set planner multipoint"""

        # Create service client
        client = self.node.create_client(SetPlanner, '/unified_planner/set_planner')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/set_planner' not available after {timeout}s")

        # Create request
        request = SetPlanner.Request()
        set_message_fields(request, {'planner_type': 4})

        # Call service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)

        # Check if call completed
        if not future.done():
            self.fail("Service call to '/unified_planner/set_planner' timed out")

        # Get response
        response = future.result()
        if response is None:
            self.fail("Service call to '/unified_planner/set_planner' failed")


        self.assertEqual(
            response.success,
            True,
            f"Field 'success' doesn't match expected value"
        )

    def test_03_generate_trajectory_multipoint(self):
        """03 Generate trajectory multipoint"""

        # Create service client
        client = self.node.create_client(TrajectoryGeneration, '/unified_planner/generate_trajectory')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/generate_trajectory' not available after {timeout}s")

        # Create request
        request = TrajectoryGeneration.Request()
        set_message_fields(request, {'start_pose': {'name': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'], 'position': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'velocity': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}, 'target_poses': [{'position': {'x': 0.5, 'y': 0.0, 'z': 0.5}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}, {'position': {'x': 0.3, 'y': 0.3, 'z': 0.6}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}]})

        # Call service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)

        # Check if call completed
        if not future.done():
            self.fail("Service call to '/unified_planner/generate_trajectory' timed out")

        # Get response
        response = future.result()
        if response is None:
            self.fail("Service call to '/unified_planner/generate_trajectory' failed")


        self.assertEqual(
            response.success,
            True,
            f"Field 'success' doesn't match expected value"
        )

    def test_04_set_planner_classic(self):
        """04 Set planner classic"""

        # Create service client
        client = self.node.create_client(SetPlanner, '/unified_planner/set_planner')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/set_planner' not available after {timeout}s")

        # Create request
        request = SetPlanner.Request()
        set_message_fields(request, {'planner_type': 0})

        # Call service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)

        # Check if call completed
        if not future.done():
            self.fail("Service call to '/unified_planner/set_planner' timed out")

        # Get response
        response = future.result()
        if response is None:
            self.fail("Service call to '/unified_planner/set_planner' failed")


        self.assertEqual(
            response.success,
            True,
            f"Field 'success' doesn't match expected value"
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

