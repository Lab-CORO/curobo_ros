"""
Test robot configurations and parameters.

Provides reusable configuration data for integration tests.
"""


class TestRobotConfig:
    """Test configuration parameters."""

    # Robot type
    ROBOT_TYPE = 'doosan_m1013'

    # Joint names for Doosan M1013
    JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

    # Number of joints
    NUM_JOINTS = 6

    # Timeout for service calls (seconds)
    SERVICE_TIMEOUT = 10.0

    # Timeout for action execution (seconds)
    ACTION_TIMEOUT = 30.0

    # Timeout for node availability (seconds)
    NODE_TIMEOUT = 5.0

    # MPC parameters
    MPC_CONVERGENCE_THRESHOLD = 0.01  # meters
    MPC_MAX_ITERATIONS = 1000
    MPC_MAX_TIME_PER_STEP = 0.1  # seconds (100ms)

    # Trajectory parameters
    TIME_DILATION_FACTOR = 0.5
    MAX_ATTEMPTS = 1
    PLANNING_TIMEOUT = 5.0

    # Collision parameters
    VOXEL_SIZE = 0.05  # meters
    COLLISION_ACTIVATION_DISTANCE = 0.025  # meters

    # Test object dimensions
    TEST_CUBOID_DIMS = [0.2, 0.2, 0.2]  # meters
    TEST_SPHERE_RADIUS = 0.1  # meters
    TEST_CYLINDER_DIMS = [0.1, 0.3]  # radius, height in meters

    @staticmethod
    def get_test_object_pose():
        """Get standard test object pose."""
        return {
            'position': {'x': 0.5, 'y': 0.0, 'z': 0.3},
            'orientation': {'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0}
        }
