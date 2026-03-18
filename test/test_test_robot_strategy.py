
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
from curobo_msgs.srv import GetRobotStrategies
from curobo_msgs.srv import SetRobotStrategy

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


    def test_01_get_robot_strategies(self):
        """01 Get robot strategies"""

        # Create service client
        client = self.node.create_client(GetRobotStrategies, '/unified_planner/get_robot_strategies')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/get_robot_strategies' not available after {timeout}s")

        # Create request
        request = GetRobotStrategies.Request()

        # Call service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)

        # Check if call completed
        if not future.done():
            self.fail("Service call to '/unified_planner/get_robot_strategies' timed out")

        # Get response
        response = future.result()
        if response is None:
            self.fail("Service call to '/unified_planner/get_robot_strategies' failed")


        self.assertEqual(
            response.success,
            True,
            f"Field 'success' doesn't match expected value"
        )

    def test_02_set_strategy_emulator(self):
        """02 Set strategy emulator"""

        # Create service client
        client = self.node.create_client(SetRobotStrategy, '/unified_planner/set_robot_strategy')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/set_robot_strategy' not available after {timeout}s")

        # Create request
        request = SetRobotStrategy.Request()
        request.robot_strategy = 1

        # Call service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)

        # Check if call completed
        if not future.done():
            self.fail("Service call to '/unified_planner/set_robot_strategy' timed out")

        # Get response
        response = future.result()
        if response is None:
            self.fail("Service call to '/unified_planner/set_robot_strategy' failed")


        self.assertEqual(
            response.success,
            True,
            f"Field 'success' doesn't match expected value"
        )

    def test_03_set_strategy_doosan(self):
        """03 Set strategy doosan"""

        # Create service client
        client = self.node.create_client(SetRobotStrategy, '/unified_planner/set_robot_strategy')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/set_robot_strategy' not available after {timeout}s")

        # Create request
        request = SetRobotStrategy.Request()
        request.robot_strategy = 0

        # Call service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)

        # Check if call completed
        if not future.done():
            self.fail("Service call to '/unified_planner/set_robot_strategy' timed out")

        # Get response
        response = future.result()
        if response is None:
            self.fail("Service call to '/unified_planner/set_robot_strategy' failed")


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
            allowable_exit_codes=[0, 1, -2, -11]  # 0: clean, 1: shutdown error, -2: SIGINT, -11: rviz2 SIGSEGV on shutdown
        )

