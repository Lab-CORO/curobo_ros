
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
from curobo_msgs.srv import WarmupIK
from curobo_msgs.srv import WarmupFK
from curobo_msgs.srv import Ik
from curobo_msgs.srv import Fk

def generate_test_description():
    """Generate launch description for test."""
    
    node_0 = LaunchNode(
        package='curobo_ros',
        executable='curobo_trajectory_planner',
        name='unified_planner',
        output='screen'
    )
    
    return LaunchDescription([
        node_0,
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


    def test_01_warmup_ik(self):
        """01 warmup IK"""

        # Create service client
        client = self.node.create_client(WarmupIK, '/unified_planner/warmup_ik')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/warmup_ik' not available after {timeout}s")

        # Create request
        request = WarmupIK.Request()
        set_message_fields(request, {'batch_size': 1})

        # Call service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)

        # Check if call completed
        if not future.done():
            self.fail("Service call to '/unified_planner/warmup_ik' timed out")

        # Get response
        response = future.result()
        if response is None:
            self.fail("Service call to '/unified_planner/warmup_ik' failed")


        self.assertEqual(
            response.success,
            True,
            f"Field 'success' doesn't match expected value"
        )

    def test_02_warmup_fk(self):
        """02 warmup FK"""

        # Create service client
        client = self.node.create_client(WarmupFK, '/unified_planner/warmup_fk')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/warmup_fk' not available after {timeout}s")

        # Create request
        request = WarmupFK.Request()
        set_message_fields(request, {'batch_size': 1})

        # Call service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)

        # Check if call completed
        if not future.done():
            self.fail("Service call to '/unified_planner/warmup_fk' timed out")

        # Get response
        response = future.result()
        if response is None:
            self.fail("Service call to '/unified_planner/warmup_fk' failed")


        self.assertEqual(
            response.success,
            True,
            f"Field 'success' doesn't match expected value"
        )

    def test_03_ik_valid_pose(self):
        """03 IK valid pose"""

        # Create service client
        client = self.node.create_client(Ik, '/unified_planner/ik')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/ik' not available after {timeout}s")

        # Create request
        request = Ik.Request()
        set_message_fields(request, {'pose': {'position': {'x': 0.5, 'y': 0.0, 'z': 0.5}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}})

        # Call service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)

        # Check if call completed
        if not future.done():
            self.fail("Service call to '/unified_planner/ik' timed out")

        # Get response
        response = future.result()
        if response is None:
            self.fail("Service call to '/unified_planner/ik' failed")


        self.assertEqual(
            response.joint_states_valid.data,
            True,
            f"Field 'data' doesn't match expected value"
        )

    def test_04_ik_invalid_pose(self):
        """04 IK invalid pose"""

        # Create service client
        client = self.node.create_client(Ik, '/unified_planner/ik')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/ik' not available after {timeout}s")

        # Create request
        request = Ik.Request()
        set_message_fields(request, {'pose': {'position': {'x': 10.0, 'y': 10.0, 'z': 10.0}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}})

        # Call service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)

        # Check if call completed
        if not future.done():
            self.fail("Service call to '/unified_planner/ik' timed out")

        # Get response
        response = future.result()
        if response is None:
            self.fail("Service call to '/unified_planner/ik' failed")


        self.assertEqual(
            response.joint_states_valid.data,
            False,
            f"Field 'data' doesn't match expected value"
        )

    def test_05_fk_valid_joints(self):
        """05 FK valid joints"""

        # Create service client
        client = self.node.create_client(Fk, '/unified_planner/fk')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/fk' not available after {timeout}s")

        # Create request
        request = Fk.Request()
        set_message_fields(request, {'joint_states': [{'position': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}]})

        # Call service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)

        # Check if call completed
        if not future.done():
            self.fail("Service call to '/unified_planner/fk' timed out")

        # Get response
        response = future.result()
        if response is None:
            self.fail("Service call to '/unified_planner/fk' failed")


@launch_testing.post_shutdown_test()
class PostShutdownTests(unittest.TestCase):
    """Post-shutdown tests to validate clean exit"""

    def test_exit_codes(self, proc_info):
        """Test that all processes exited without critical errors"""
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, 1, -2, -6, -11]  # 0: clean, 1: shutdown error, -2: SIGINT, -6: rviz2 SIGABRT, -11: rviz2 SIGSEGV on shutdown
        )

