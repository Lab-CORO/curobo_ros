
"""
Auto-generated test file
Generated from YAML specification
DO NOT EDIT - Changes will be overwritten
"""

import unittest
import pytest
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
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
from curobo_msgs.srv import SetRobotStrategy
from curobo_msgs.action import SendTrajectory
from std_srvs.srv import Trigger

@launch_testing.ready_to_test_action_timeout(90.0)
@pytest.mark.launch_test
def generate_test_description():
    """Generate launch description for test."""
    
    launch_file_0_path = PathJoinSubstitution([
        FindPackageShare('curobo_ros'),
        'launch',
        'gen_traj_test.launch.py'
    ])

    launch_file_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_0_path)
    )
    
    return LaunchDescription([
        launch_file_0,
        launch_testing.util.KeepAliveProc(),
        launch_testing.actions.ReadyToTest(),
    ]), locals()



class GeneratedTestSuite(unittest.TestCase):
    """Generated test class"""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls._wait_until_ready()

    @classmethod
    def _wait_until_ready(cls):
        """Block until '/unified_planner/generate_trajectory' is advertised, or fail the suite.

        Graph introspection, never a call. A node that builds heavy
        state in its constructor advertises nothing until that work
        is done, so the service appearing in the graph *is* the
        readiness signal -- and because nothing is invoked, a service
        whose callback is broken cannot make this hang. Pick a
        service the node creates late in its startup.

        This replaces guessing with `startup_delay`, which raced
        machine load and failed on whichever suite happened to be
        slowest that run.
        """
        node = rclpy.create_node('readiness_probe')
        try:
            deadline = time.monotonic() + 180.0
            while time.monotonic() < deadline:
                advertised = [
                    name for name, _
                    in node.get_service_names_and_types()
                ]
                if '/unified_planner/generate_trajectory' in advertised:
                    return
                time.sleep(0.5)
            raise AssertionError(
                "System under test never became ready: "
                "'/unified_planner/generate_trajectory' was not advertised within 180.0s. "
                "Raise 'ready_wait' if startup is legitimately "
                "slower, otherwise the node failed to start -- "
                "check the launch output above.")
        finally:
            node.destroy_node()

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

    def test_04_set_robot_strategy_emulator(self):
        """04 Set robot strategy emulator"""

        # Create service client
        client = self.node.create_client(SetRobotStrategy, '/unified_planner/set_robot_strategy')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/set_robot_strategy' not available after {timeout}s")

        # Create request
        request = SetRobotStrategy.Request()
        set_message_fields(request, {'robot_strategy': 'emulator'})

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

    def test_05_set_planner_classic(self):
        """05 Set planner classic"""

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

    def test_06_generate_trajectory_classic_preview(self):
        """06 Generate trajectory classic preview"""

        # Create service client
        client = self.node.create_client(TrajectoryGeneration, '/unified_planner/generate_trajectory')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/generate_trajectory' not available after {timeout}s")

        # Create request
        request = TrajectoryGeneration.Request()
        set_message_fields(request, {'target_pose': {'position': {'x': 0.5, 'y': 0.0, 'z': 0.5}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}})

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

    def test_07_execute_classic_cached(self):
        """07 Execute classic cached"""

        # Storage for feedback messages
        self.received_feedback = []

        def feedback_callback(feedback_msg):
            self.received_feedback.append(feedback_msg.feedback)

        # Create action client
        client = ActionClient(self.node, SendTrajectory, '/unified_planner/execute_trajectory')

        # Wait for the action server to be available
        timeout = 90.0
        if not client.wait_for_server(timeout_sec=timeout):
            self.fail(f"Action server '/unified_planner/execute_trajectory' not available after {timeout}s")

        # Create and send the goal
        goal = SendTrajectory.Goal()
        set_message_fields(goal, {'target_pose': {'position': {'x': 0.5, 'y': 0.0, 'z': 0.5}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}})

        send_future = client.send_goal_async(goal, feedback_callback=feedback_callback)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=timeout)

        goal_handle = send_future.result()
        if goal_handle is None:
            self.fail("Goal request to '/unified_planner/execute_trajectory' timed out")
        self.assertTrue(goal_handle.accepted, "Goal was rejected by '/unified_planner/execute_trajectory'")

        # Wait for the result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=timeout)

        if result_future.result() is None:
            self.fail("Action '/unified_planner/execute_trajectory' did not return a result in time")

        result = result_future.result().result


        self.assertEqual(
            result.success,
            True,
            f"Field 'success' doesn't match expected value"
        )

    def test_08_clear_trajectory(self):
        """08 Clear trajectory"""

        # Create service client
        client = self.node.create_client(Trigger, '/unified_planner/clear_trajectory')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/clear_trajectory' not available after {timeout}s")

        # Create request
        request = Trigger.Request()
        set_message_fields(request, {})

        # Call service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)

        # Check if call completed
        if not future.done():
            self.fail("Service call to '/unified_planner/clear_trajectory' timed out")

        # Get response
        response = future.result()
        if response is None:
            self.fail("Service call to '/unified_planner/clear_trajectory' failed")


        self.assertEqual(
            response.success,
            True,
            f"Field 'success' doesn't match expected value"
        )

    def test_09_set_planner_mpc(self):
        """09 Set planner MPC"""

        # Create service client
        client = self.node.create_client(SetPlanner, '/unified_planner/set_planner')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/set_planner' not available after {timeout}s")

        # Create request
        request = SetPlanner.Request()
        set_message_fields(request, {'planner_type': 1})

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

        self.assertEqual(
            response.current_planner,
            'Model Predictive Control (MPC)',
            f"Field 'current_planner' doesn't match expected value"
        )

    def test_10_execute_mpc_reactive(self):
        """10 Execute MPC reactive"""

        # Storage for feedback messages
        self.received_feedback = []

        def feedback_callback(feedback_msg):
            self.received_feedback.append(feedback_msg.feedback)

        # Create action client
        client = ActionClient(self.node, SendTrajectory, '/unified_planner/execute_trajectory')

        # Wait for the action server to be available
        timeout = 90.0
        if not client.wait_for_server(timeout_sec=timeout):
            self.fail(f"Action server '/unified_planner/execute_trajectory' not available after {timeout}s")

        # Create and send the goal
        goal = SendTrajectory.Goal()
        set_message_fields(goal, {'target_pose': {'position': {'x': 0.627, 'y': -0.005, 'z': 0.751}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}})

        send_future = client.send_goal_async(goal, feedback_callback=feedback_callback)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=timeout)

        goal_handle = send_future.result()
        if goal_handle is None:
            self.fail("Goal request to '/unified_planner/execute_trajectory' timed out")
        self.assertTrue(goal_handle.accepted, "Goal was rejected by '/unified_planner/execute_trajectory'")

        # Reactive action: it does NOT terminate on its own. Collect feedback for
        # a while, then cancel it.
        _deadline = time.time() + 6.0
        while time.time() < _deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self.node, cancel_future, timeout_sec=timeout)

        # Wait for the result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=timeout)

        if result_future.result() is None:
            self.fail("Action '/unified_planner/execute_trajectory' did not return a result in time")

        result = result_future.result().result


        self.assertGreater(
            len(self.received_feedback),
            0,
            "Reactive action published no feedback (servo loop did not run)"
        )

        self.assertEqual(
            result.success,
            False,
            f"Field 'success' doesn't match expected value"
        )

    def test_11_set_planner_lbfgs(self):
        """11 Set planner LBFGS"""

        # Create service client
        client = self.node.create_client(SetPlanner, '/unified_planner/set_planner')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/set_planner' not available after {timeout}s")

        # Create request
        request = SetPlanner.Request()
        set_message_fields(request, {'planner_type': 2})

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

        self.assertEqual(
            response.current_planner,
            'LBFGS Model Predictive Control',
            f"Field 'current_planner' doesn't match expected value"
        )

    def test_12_execute_lbfgs_reactive(self):
        """12 Execute LBFGS reactive"""

        # Storage for feedback messages
        self.received_feedback = []

        def feedback_callback(feedback_msg):
            self.received_feedback.append(feedback_msg.feedback)

        # Create action client
        client = ActionClient(self.node, SendTrajectory, '/unified_planner/execute_trajectory')

        # Wait for the action server to be available
        timeout = 90.0
        if not client.wait_for_server(timeout_sec=timeout):
            self.fail(f"Action server '/unified_planner/execute_trajectory' not available after {timeout}s")

        # Create and send the goal
        goal = SendTrajectory.Goal()
        set_message_fields(goal, {'target_pose': {'position': {'x': 0.627, 'y': -0.005, 'z': 0.751}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}})

        send_future = client.send_goal_async(goal, feedback_callback=feedback_callback)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=timeout)

        goal_handle = send_future.result()
        if goal_handle is None:
            self.fail("Goal request to '/unified_planner/execute_trajectory' timed out")
        self.assertTrue(goal_handle.accepted, "Goal was rejected by '/unified_planner/execute_trajectory'")

        # Reactive action: it does NOT terminate on its own. Collect feedback for
        # a while, then cancel it.
        _deadline = time.time() + 6.0
        while time.time() < _deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self.node, cancel_future, timeout_sec=timeout)

        # Wait for the result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=timeout)

        if result_future.result() is None:
            self.fail("Action '/unified_planner/execute_trajectory' did not return a result in time")

        result = result_future.result().result


        self.assertGreater(
            len(self.received_feedback),
            0,
            "Reactive action published no feedback (servo loop did not run)"
        )

        self.assertEqual(
            result.success,
            False,
            f"Field 'success' doesn't match expected value"
        )

    def test_13_set_planner_classic(self):
        """13 Set planner classic"""

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

    def test_14_generate_trajectory_with_constraint(self):
        """14 Generate trajectory with constraint"""

        # Create service client
        client = self.node.create_client(TrajectoryGeneration, '/unified_planner/generate_trajectory')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/generate_trajectory' not available after {timeout}s")

        # Create request
        request = TrajectoryGeneration.Request()
        set_message_fields(request, {'target_pose': {'position': {'x': 0.5, 'y': 0.0, 'z': 0.5}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}, 'trajectory_constraints': [0, 0, 1, 0, 0, 0]})

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

    def test_15_set_planner_retarget(self):
        """15 Set planner retarget"""

        # Create service client
        client = self.node.create_client(SetPlanner, '/unified_planner/set_planner')

        # Wait for service to be available
        timeout = 10.0
        if not client.wait_for_service(timeout_sec=timeout):
            self.fail(f"Service '/unified_planner/set_planner' not available after {timeout}s")

        # Create request
        request = SetPlanner.Request()
        set_message_fields(request, {'planner_type': 6})

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

        self.assertEqual(
            response.current_planner,
            'Motion Retargeting (Teleop)',
            f"Field 'current_planner' doesn't match expected value"
        )

    def test_16_execute_retarget_reactive(self):
        """16 Execute retarget reactive"""

        # Storage for feedback messages
        self.received_feedback = []

        def feedback_callback(feedback_msg):
            self.received_feedback.append(feedback_msg.feedback)

        # Create action client
        client = ActionClient(self.node, SendTrajectory, '/unified_planner/execute_trajectory')

        # Wait for the action server to be available
        timeout = 90.0
        if not client.wait_for_server(timeout_sec=timeout):
            self.fail(f"Action server '/unified_planner/execute_trajectory' not available after {timeout}s")

        # Create and send the goal
        goal = SendTrajectory.Goal()
        set_message_fields(goal, {'target_pose': {'position': {'x': 0.627, 'y': -0.005, 'z': 0.751}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}})

        send_future = client.send_goal_async(goal, feedback_callback=feedback_callback)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=timeout)

        goal_handle = send_future.result()
        if goal_handle is None:
            self.fail("Goal request to '/unified_planner/execute_trajectory' timed out")
        self.assertTrue(goal_handle.accepted, "Goal was rejected by '/unified_planner/execute_trajectory'")

        # Reactive action: it does NOT terminate on its own. Collect feedback for
        # a while, then cancel it.
        _deadline = time.time() + 6.0
        while time.time() < _deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self.node, cancel_future, timeout_sec=timeout)

        # Wait for the result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=timeout)

        if result_future.result() is None:
            self.fail("Action '/unified_planner/execute_trajectory' did not return a result in time")

        result = result_future.result().result


        self.assertGreater(
            len(self.received_feedback),
            0,
            "Reactive action published no feedback (servo loop did not run)"
        )

        self.assertEqual(
            result.success,
            False,
            f"Field 'success' doesn't match expected value"
        )

@launch_testing.post_shutdown_test()
class PostShutdownTests(unittest.TestCase):
    """Post-shutdown tests to validate clean exit"""

    def test_exit_codes(self, proc_info):
        """Test that all processes exited without critical errors"""
        launch_testing.asserts.assertExitCodes(proc_info)

