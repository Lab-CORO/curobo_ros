
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


    def test_01_set_planner_joint_space(self):
        """01 Set planner joint space"""

        # Create service client
        client = self.node.create_client(SetPlanner, '/unified_planner/set_planner')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/set_planner' not available after {timeout}s")

        # Create request
        request = SetPlanner.Request()
        request.planner_type = 5

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

    def test_02_generate_trajectory_joint_space(self):
        """02 Generate trajectory joint space"""

        # Create service client
        client = self.node.create_client(TrajectoryGeneration, '/unified_planner/generate_trajectory')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/generate_trajectory' not available after {timeout}s")

        # Create request
        request = TrajectoryGeneration.Request()
        request.target_joint_positions = [0.5, -0.5, 0.5, -0.5, 0.5, -0.5]
        request.start_pose.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        request.start_pose.position = [0.1, 0.2, 0.3, 0.0, 0.0, 0.0]
        request.start_pose.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

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

    def test_03_set_planner_classic(self):
        """03 Set planner classic"""

        # Create service client
        client = self.node.create_client(SetPlanner, '/unified_planner/set_planner')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/set_planner' not available after {timeout}s")

        # Create request
        request = SetPlanner.Request()
        request.planner_type = 0

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

    def test_04_generate_trajectory_cartesian(self):
        """04 Generate trajectory cartesian"""

        # Create service client
        client = self.node.create_client(TrajectoryGeneration, '/unified_planner/generate_trajectory')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/generate_trajectory' not available after {timeout}s")

        # Create request
        request = TrajectoryGeneration.Request()
        request.start_pose.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        request.start_pose.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        request.start_pose.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        request.target_pose.position.x = 0.5
        request.target_pose.position.y = 0.0
        request.target_pose.position.z = 0.5
        request.target_pose.orientation.x = 0.0
        request.target_pose.orientation.y = 0.0
        request.target_pose.orientation.z = 0.0
        request.target_pose.orientation.w = 1.0

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

    def test_05_generate_trajectory_invalid_target(self):
        """05 Generate trajectory invalid target"""

        # Create service client
        client = self.node.create_client(TrajectoryGeneration, '/unified_planner/generate_trajectory')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/generate_trajectory' not available after {timeout}s")

        # Create request
        request = TrajectoryGeneration.Request()
        request.start_pose.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        request.start_pose.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        request.start_pose.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        request.target_pose.position.x = 10.0
        request.target_pose.position.y = 10.0
        request.target_pose.position.z = 10.0
        request.target_pose.orientation.x = 0.0
        request.target_pose.orientation.y = 0.0
        request.target_pose.orientation.z = 0.0
        request.target_pose.orientation.w = 1.0

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
            False,
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

