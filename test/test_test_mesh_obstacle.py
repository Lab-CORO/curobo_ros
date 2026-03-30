
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
from curobo_msgs.srv import SetCollisionCache
from std_srvs.srv import Trigger
from curobo_msgs.srv import AddObject
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


    def test_01_set_collision_cache(self):
        """01 set collision cache"""

        # Create service client
        client = self.node.create_client(SetCollisionCache, '/unified_planner/set_collision_cache')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/set_collision_cache' not available after {timeout}s")

        # Create request
        request = SetCollisionCache.Request()
        set_message_fields(request, {'obb': 136, 'mesh': 1, 'blox': -1})

        # Call service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)

        # Check if call completed
        if not future.done():
            self.fail("Service call to '/unified_planner/set_collision_cache' timed out")

        # Get response
        response = future.result()
        if response is None:
            self.fail("Service call to '/unified_planner/set_collision_cache' failed")


        self.assertEqual(
            response.success,
            True,
            f"Field 'success' doesn't match expected value"
        )

        self.assertEqual(
            response.obb_cache,
            136,
            f"Field 'obb_cache' doesn't match expected value"
        )

        self.assertEqual(
            response.mesh_cache,
            1,
            f"Field 'mesh_cache' doesn't match expected value"
        )

    def test_02_reload_motiongen_with_new_cache(self):
        """02 reload MotionGen with new cache"""

        # Create service client
        client = self.node.create_client(Trigger, '/unified_planner/update_motion_gen_config')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/update_motion_gen_config' not available after {timeout}s")

        # Create request
        request = Trigger.Request()
        set_message_fields(request, {})

        # Call service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)

        # Check if call completed
        if not future.done():
            self.fail("Service call to '/unified_planner/update_motion_gen_config' timed out")

        # Get response
        response = future.result()
        if response is None:
            self.fail("Service call to '/unified_planner/update_motion_gen_config' failed")


        self.assertEqual(
            response.success,
            True,
            f"Field 'success' doesn't match expected value"
        )

    def test_03_add_rubber_duck_mesh_at_origin(self):
        """03 add Rubber Duck mesh at origin"""

        # Create service client
        client = self.node.create_client(AddObject, '/unified_planner/add_object')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/add_object' not available after {timeout}s")

        # Create request
        request = AddObject.Request()
        set_message_fields(request, {'type': 4, 'name': 'rubber_duck', 'mesh_file_path': '/home/ros2_ws/src/curobo_ros/config/config_test/Rubber_Duck.stl', 'pose': {'position': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}, 'dimensions': {'x': 0.01, 'y': 0.01, 'z': 0.01}, 'color': {'r': 1.0, 'g': 0.8, 'b': 0.0, 'a': 1.0}})

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
            "Object 'rubber_duck' added successfully (136 cuboids, 1 mesh in world)",
            f"Field 'message' doesn't match expected value"
        )

    def test_04_trajectory_blocked_by_duck(self):
        """04 trajectory blocked by duck"""

        # Create service client
        client = self.node.create_client(TrajectoryGeneration, '/unified_planner/generate_trajectory')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/generate_trajectory' not available after {timeout}s")

        # Create request
        request = TrajectoryGeneration.Request()
        set_message_fields(request, {'target_pose': {'position': {'x': 0.1, 'y': 0.0, 'z': 0.1}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}})

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
            allowable_exit_codes=[0, 1, -2, -6, -11]  # 0: clean, 1: shutdown error, -2: SIGINT, -6: rviz2 SIGABRT, -11: rviz2 SIGSEGV on shutdown
        )

