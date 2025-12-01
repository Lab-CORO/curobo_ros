"""
Test poses and joint states for integration tests.

Provides reusable test data for FK, IK, and trajectory generation tests.
"""

from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState


class TestPoses:
    """Reusable test poses for Doosan M1013 robot."""

    @staticmethod
    def home_pose() -> Pose:
        """Home position pose (all joints at 0)."""
        pose = Pose()
        pose.position = Point(x=0.5, y=0.0, z=0.5)
        pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        return pose

    @staticmethod
    def reach_pose_1() -> Pose:
        """Reachable pose 1 - Front reach."""
        pose = Pose()
        pose.position = Point(x=0.6, y=0.2, z=0.4)
        pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        return pose

    @staticmethod
    def reach_pose_2() -> Pose:
        """Reachable pose 2 - Side reach."""
        pose = Pose()
        pose.position = Point(x=0.4, y=0.3, z=0.5)
        pose.orientation = Quaternion(w=0.707, x=0.0, y=0.0, z=0.707)
        return pose

    @staticmethod
    def reach_pose_3() -> Pose:
        """Reachable pose 3 - High reach."""
        pose = Pose()
        pose.position = Point(x=0.5, y=0.0, z=0.7)
        pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        return pose

    @staticmethod
    def unreachable_pose() -> Pose:
        """Unreachable pose - Too far."""
        pose = Pose()
        pose.position = Point(x=5.0, y=5.0, z=5.0)
        pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        return pose

    @staticmethod
    def collision_pose() -> Pose:
        """Pose that would cause collision with ground."""
        pose = Pose()
        pose.position = Point(x=0.5, y=0.0, z=-0.2)  # Below ground
        pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        return pose


class TestJointStates:
    """Reusable joint states for Doosan M1013 robot."""

    @staticmethod
    def home_state() -> JointState:
        """Home joint configuration (all zeros)."""
        js = JointState()
        js.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        js.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        return js

    @staticmethod
    def valid_state_1() -> JointState:
        """Valid joint configuration 1."""
        js = JointState()
        js.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        js.position = [0.5, -0.3, 0.8, 0.0, 1.2, 0.0]
        return js

    @staticmethod
    def valid_state_2() -> JointState:
        """Valid joint configuration 2."""
        js = JointState()
        js.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        js.position = [-0.5, 0.5, -0.5, 1.0, -1.0, 0.5]
        return js

    @staticmethod
    def batch_states(n: int = 10) -> list:
        """Generate batch of valid joint states."""
        import math
        states = []
        for i in range(n):
            js = JointState()
            js.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
            # Generate varying configurations
            angle = 2 * math.pi * i / n
            js.position = [
                0.5 * math.sin(angle),
                0.3 * math.cos(angle),
                0.4 * math.sin(2 * angle),
                0.2 * math.cos(2 * angle),
                0.5 * math.sin(3 * angle),
                0.3 * math.cos(3 * angle),
            ]
            states.append(js)
        return states
