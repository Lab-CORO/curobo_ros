"""
Configuration pytest pour les tests d'intégration CuRobo ROS

Ce fichier contient les fixtures partagées entre tous les tests d'intégration.
"""

import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import time
from pathlib import Path


@pytest.fixture(scope="session")
def rclpy_init():
    """Initialize ROS 2 context for the entire test session"""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def ros_executor(rclpy_init):
    """Provide a ROS 2 executor for running nodes"""
    executor = MultiThreadedExecutor()

    def spin_executor():
        try:
            executor.spin()
        except Exception:
            pass

    spin_thread = threading.Thread(target=spin_executor, daemon=True)
    spin_thread.start()

    yield executor

    executor.shutdown()
    spin_thread.join(timeout=1.0)


@pytest.fixture
def test_node(ros_executor):
    """Create a test node for making service calls and subscribing to topics"""
    node = Node('test_node')
    ros_executor.add_node(node)
    yield node
    node.destroy_node()


@pytest.fixture
def rosbag_base_path():
    """Return the base path to test rosbags"""
    return Path(__file__).parent.parent / "rosbags"


@pytest.fixture
def test_poses():
    """Provide common test poses for IK/FK/trajectory tests"""
    from geometry_msgs.msg import Pose, Point, Quaternion

    poses = {
        'home': Pose(
            position=Point(x=0.3, y=0.0, z=0.4),
            orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)
        ),
        'pick': Pose(
            position=Point(x=0.5, y=0.2, z=0.1),
            orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)
        ),
        'place': Pose(
            position=Point(x=0.3, y=-0.3, z=0.2),
            orientation=Quaternion(x=0.0, y=0.707, z=0.0, w=0.707)
        ),
        'unreachable': Pose(
            position=Point(x=2.0, y=2.0, z=2.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
    }

    return poses


@pytest.fixture
def test_obstacles():
    """Provide common test obstacles for collision testing"""
    obstacles = {
        'box_small': {
            'name': 'test_box_1',
            'type': 'box',
            'dimensions': [0.1, 0.1, 0.1],
            'position': [0.5, 0.0, 0.3],
            'orientation': [0.0, 0.0, 0.0, 1.0]
        },
        'box_large': {
            'name': 'test_box_2',
            'type': 'box',
            'dimensions': [0.3, 0.3, 0.2],
            'position': [0.4, 0.3, 0.2],
            'orientation': [0.0, 0.0, 0.0, 1.0]
        },
        'cylinder': {
            'name': 'test_cylinder',
            'type': 'cylinder',
            'radius': 0.05,
            'height': 0.2,
            'position': [0.3, -0.2, 0.1],
            'orientation': [0.0, 0.0, 0.0, 1.0]
        },
        'sphere': {
            'name': 'test_sphere',
            'type': 'sphere',
            'radius': 0.08,
            'position': [0.6, 0.0, 0.4],
            'orientation': [0.0, 0.0, 0.0, 1.0]
        }
    }

    return obstacles


@pytest.fixture
def timeout_default():
    """Default timeout for service calls (seconds)"""
    return 10.0


@pytest.fixture
def test_config():
    """Common test configuration parameters"""
    config = {
        'robot_type': 'doosan_m1013',
        'base_link': 'base_0',
        'max_attempts': 3,
        'timeout': 5.0,
        'voxel_size': 0.05,
        'collision_activation_distance': 0.025,
        'position_tolerance': 0.01,  # meters
        'orientation_tolerance': 0.05,  # radians
        'joint_tolerance': 0.05,  # radians
    }

    return config


class ServiceClientHelper:
    """Helper class for making service calls with timeout and retry"""

    def __init__(self, node, service_type, service_name, timeout=10.0):
        self.node = node
        self.client = node.create_client(service_type, service_name)
        self.timeout = timeout
        self.service_name = service_name

    def wait_for_service(self, timeout=None):
        """Wait for service to become available"""
        timeout = timeout or self.timeout
        return self.client.wait_for_service(timeout_sec=timeout)

    def call(self, request, timeout=None):
        """Call service and wait for response"""
        timeout = timeout or self.timeout

        if not self.wait_for_service(timeout):
            raise TimeoutError(f"Service {self.service_name} not available after {timeout}s")

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)

        if not future.done():
            raise TimeoutError(f"Service call to {self.service_name} timed out after {timeout}s")

        return future.result()


class TopicSubscriberHelper:
    """Helper class for subscribing to topics and collecting messages"""

    def __init__(self, node, msg_type, topic_name, queue_size=10):
        self.node = node
        self.messages = []
        self.topic_name = topic_name
        self.subscription = node.create_subscription(
            msg_type,
            topic_name,
            self._callback,
            queue_size
        )

    def _callback(self, msg):
        """Store received messages"""
        self.messages.append(msg)

    def wait_for_messages(self, count=1, timeout=5.0):
        """Wait until at least 'count' messages are received"""
        start_time = time.time()
        rate = self.node.create_rate(10)  # 10 Hz

        while len(self.messages) < count:
            if time.time() - start_time > timeout:
                raise TimeoutError(
                    f"Only received {len(self.messages)}/{count} messages on {self.topic_name} "
                    f"after {timeout}s"
                )
            rclpy.spin_once(self.node, timeout_sec=0.1)
            rate.sleep()

        return self.messages

    def clear(self):
        """Clear collected messages"""
        self.messages = []

    def get_latest(self):
        """Get most recent message"""
        return self.messages[-1] if self.messages else None


@pytest.fixture
def service_helper_factory(test_node, timeout_default):
    """Factory for creating service client helpers"""
    def create_helper(service_type, service_name, timeout=None):
        timeout = timeout or timeout_default
        return ServiceClientHelper(test_node, service_type, service_name, timeout)

    return create_helper


@pytest.fixture
def topic_helper_factory(test_node):
    """Factory for creating topic subscriber helpers"""
    def create_helper(msg_type, topic_name, queue_size=10):
        return TopicSubscriberHelper(test_node, msg_type, topic_name, queue_size)

    return create_helper


@pytest.fixture
def launch_nodes():
    """
    Context manager for launching ROS nodes during tests

    Usage:
        with launch_nodes(['curobo_gen_traj', 'curobo_ik']):
            # Run tests
            pass
    """
    from contextlib import contextmanager
    import subprocess

    @contextmanager
    def _launch(node_names, params=None):
        processes = []
        params = params or {}

        try:
            # Launch each node
            for node_name in node_names:
                cmd = ['ros2', 'run', 'curobo_ros', node_name]

                # Add parameters if specified
                if node_name in params:
                    for key, value in params[node_name].items():
                        cmd.extend(['-p', f'{key}:={value}'])

                process = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
                processes.append((node_name, process))

            # Wait for nodes to initialize
            time.sleep(3.0)

            yield processes

        finally:
            # Terminate all launched nodes
            for node_name, process in processes:
                process.terminate()
                try:
                    process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    process.kill()

    return _launch


@pytest.fixture
def performance_metrics():
    """Helper for collecting performance metrics during tests"""
    class PerformanceMetrics:
        def __init__(self):
            self.timings = {}
            self.counters = {}

        def start_timer(self, name):
            """Start a named timer"""
            self.timings[name] = {'start': time.time(), 'end': None, 'duration': None}

        def stop_timer(self, name):
            """Stop a named timer and calculate duration"""
            if name in self.timings:
                self.timings[name]['end'] = time.time()
                self.timings[name]['duration'] = (
                    self.timings[name]['end'] - self.timings[name]['start']
                )

        def get_duration(self, name):
            """Get duration of a completed timer"""
            return self.timings.get(name, {}).get('duration')

        def increment(self, name, value=1):
            """Increment a counter"""
            self.counters[name] = self.counters.get(name, 0) + value

        def get_counter(self, name):
            """Get counter value"""
            return self.counters.get(name, 0)

        def report(self):
            """Generate a report of collected metrics"""
            report = "Performance Metrics:\n"
            report += "=" * 50 + "\n"

            if self.timings:
                report += "Timings:\n"
                for name, data in self.timings.items():
                    duration = data.get('duration', 'N/A')
                    report += f"  {name}: {duration:.3f}s\n" if duration != 'N/A' else f"  {name}: {duration}\n"

            if self.counters:
                report += "\nCounters:\n"
                for name, value in self.counters.items():
                    report += f"  {name}: {value}\n"

            return report

    return PerformanceMetrics()
