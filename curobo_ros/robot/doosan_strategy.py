from dsr_msgs2.msg import SpeedjRtStream
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup



from curobo_ros.robot.joint_control_strategy import JointCommandStrategy, RobotState
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from std_msgs.msg import Float32
import numpy as np 


class DoosanControl(JointCommandStrategy):
    '''
    This class is a strategie to control doosan robot with motion_gen
    '''

    def __init__(self, node, dt):
        super().__init__(node, dt)
        # create a publisher
        self.pub_speed_command = node.create_publisher(SpeedjRtStream, '/dsr01/speedj_rt_stream', 10)
        self.pub_trajectory = node.create_publisher(JointTrajectory, '/leeloo/execute_trajectory', 10)
        self.sub_trajectory_state = node.create_subscription(Float32, "/leeloo/trajectory_state", self.callback_trajectory_state, 10, callback_group = MutuallyExclusiveCallbackGroup())
        self.node = node
        # Creatre a timer at dt
        self.time_dilation_factor = 0.01 #dt
        # self.timer = node.create_timer(0.01, self.send_command)
        self.vel_command = []
        self.accel_command = []
        self.command_index = 0
        self.robot_state = RobotState.IDLE
        self.trajectory_progression = 0.0

    def send_trajectrory(self, data):

        self.robot_state = RobotState.RUNNING
        joint_trajectory_msg = JointTrajectory()

        # Set joint names
        joint_trajectory_msg.joint_names = self.joint_names

        # Create a list of JointTrajectoryPoints for every position in the JointState
        previous_pose = []
        next_pose = []
        # If no traj to do, traj is done.
        if(len(self.position_command) == 0):
            self.trajectory_progression = 1.0

        for i in range(len(self.position_command)):
           
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract the i-th positions, velocities, and accelerations
            joint_trajectory_point.positions = self.position_command[i]
            joint_trajectory_point.velocities = self.vel_command[i]
            joint_trajectory_point.accelerations = self.accel_command[i]

            # Set efforts to an empty list (can be customized later)
            joint_trajectory_point.effort = []

            # Set the time_from_start for this point (incremented by time_step for each point)
            joint_trajectory_point.time_from_start = Duration(sec=int(self.time_dilation_factor * i),
                                                            nanosec=int((self.time_dilation_factor * i % 1) * 1e9))

            # Add the point to the trajectory message
            joint_trajectory_msg.points.append(joint_trajectory_point)

        self.pub_trajectory.publish(joint_trajectory_msg)


    def send_command(self):
        # Need to remove

        if self.command_index >= len(self.vel_command):
            self.robot_state = RobotState.IDLE
            self.command_index = 0
            self.vel_command = []
            self.accel_command = []

        elif(not self.get_send_to_robot()):
            self.robot_state = RobotState.IDLE

        else:
            traj = JointTrajectory()

            for vel in self.vel_command:
                pt = JointTrajectoryPoint()
                pt.velocities = vel
                pt.time_from_start = 0.01
                traj.append(traj)
            self.pub_trajectory.publish(traj)
        #   self.robot_state = RobotState.RUNNING
        return
    
    def get_joint_pose(self, node):
        ret, joint_pose_msg = wait_for_message(JointState, node, "/dsr01/joint_states", time_to_wait = 0.1)

        if ret:
            joint_pose = [joint_pose_msg.position[0], 
            joint_pose_msg.position[1], 
            joint_pose_msg.position[4],
            joint_pose_msg.position[2],
            joint_pose_msg.position[3], 
            joint_pose_msg.position[5]] # Stupidities from doosan cf joint_states msg
            return joint_pose
        else:
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def stop_robot(self):
        # set var to 0
        self.vel_command = []
        self.position_command = []
        self.accel_command = []
        self.command_index = 0
        self.trajectory_progression = 0.0
        self.robot_state = RobotState.STOPPED

        # send msg to robot
        msg = SpeedjRtStream()
        msg.vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.acc = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.time = 0.0
        self.pub_speed_command.publish(msg)
        
    def callback_trajectory_state(self, msg):
        self.trajectory_progression = msg.data 

    def get_progression(self):
        return self.trajectory_progression