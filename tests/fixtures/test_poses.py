"""Test poses and joint states for integration tests."""

from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState

class TestPoses:
    """Reusable test poses for Doosan M1013 robot."""
    
    @staticmethod
    def home_pose() -> Pose:
        pose = Pose()
        pose.position = Point(x=0.5, y=0.0, z=0.5)
        pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        return pose
    
    @staticmethod
    def reach_pose_1() -> Pose:
        pose = Pose()
        pose.position = Point(x=0.6, y=0.2, z=0.4)
        pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        return pose

class TestJointStates:
    """Reusable joint states for Doosan M1013 robot."""
    
    @staticmethod
    def home_state() -> JointState:
        js = JointState()
        js.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        js.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        return js
