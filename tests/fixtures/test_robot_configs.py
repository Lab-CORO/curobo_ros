"""Test robot configurations and parameters."""

class TestRobotConfig:
    """Test configuration parameters."""
    ROBOT_TYPE = 'doosan_m1013'
    JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    NUM_JOINTS = 6
    SERVICE_TIMEOUT = 10.0
    ACTION_TIMEOUT = 30.0
    MPC_CONVERGENCE_THRESHOLD = 0.01
    TEST_CUBOID_DIMS = [0.2, 0.2, 0.2]
