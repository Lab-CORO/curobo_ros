from curobo_ros.robot.joint_control_strategy import JointCommandStrategy, RobotState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class GhostStrategy(JointCommandStrategy):
    '''
    This class is a strategie to control doosan robot with motion_gen
    '''

    def __init__(self, node, dt):
        # create a publisher
        self.pub_command = node.create_publisher(JointTrajectory, 'trajectory', 10)
        self.position_command = []
        self.vel_command = []
        self.accel_command = []
        self.joint_names = []
        self.command_index = 0
        self.dt = dt
        self.robot_state = RobotState.IDLE

    def set_joint_name(names):
        self.joint_names = names

    def send_command(self):
        """
        Convert CuRobo JointState to ROS2 JointTrajectory message with multiple points.

        Args:
            joint_state (JointState): CuRobo JointState object that may contain multiple time steps.
            time_step (float): Time between each trajectory point in seconds.

        Returns:
            JointTrajectory: A ROS2 JointTrajectory message.
        """
       
        self.robot_state = RobotState.RUNNING
        joint_trajectory_msg = JointTrajectory()

        # Set joint names
        joint_trajectory_msg.joint_names = self.joint_names

            # Create a list of JointTrajectoryPoints for every position in the JointState
        for i in range(len(self.position_command)):
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract the i-th positions, velocities, and accelerations
            joint_trajectory_point.positions = self.position_command[i]
            joint_trajectory_point.velocities = self.vel_command[i]
            joint_trajectory_point.accelerations = self.accel_command[i]

            # Set efforts to an empty list (can be customized later)
            joint_trajectory_point.effort = []

            # Set the time_from_start for this point (incremented by time_step for each point)
            joint_trajectory_point.time_from_start = Duration(sec=int(self.dt * i),
                                                            nanosec=int((self.dt * i % 1) * 1e9))

            # Add the point to the trajectory message
            joint_trajectory_msg.points.append(joint_trajectory_point)

        self.pub_command.publish(joint_trajectory_msg)

