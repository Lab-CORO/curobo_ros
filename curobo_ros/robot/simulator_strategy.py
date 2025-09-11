
from curobo_ros.robot.joint_control_strategy import JointCommandStrategy, RobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class SimulatorControl(JointCommandStrategy):


    '''
    This class is a strategie to control doosan robot with motion_gen
    '''

    def __init__(self, node, dt):
        super().__init__(node, dt)
        # create a publisher
        self.pub_trajectory = node.create_publisher(JointState, '/joint_command', 10)
        self.sub_joint_state = node.create_subscription(JointState, "/joint_states", self.callback_joint_pose, 10, callback_group = MutuallyExclusiveCallbackGroup())
        self.node = node
        self.dt = 0.02 #dt is defined by curobo 
        self.position_command = []
        self.vel_command = []
        self.accel_command = []
        self.command_index = 0
        self.robot_state = RobotState.IDLE
        self.trajectory_progression = 0.0
        self.joint_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

        # Create a timer that send commands to simulator
        self.timer_send_trajectory = self.node.create_timer(dt, self.send_trajectrory)

    def send_trajectrory(self):
        
        if(len(self.position_command) == 0):
            self.trajectory_progression = 1.0
            return

        self.robot_state = RobotState.RUNNING
        joint_command_msg = JointState()

        # Set joint names
        joint_command_msg.name = self.joint_name

        # ros2 topic pub /joint_command sensor_msgs/msg/JointState "{ name: ['joint3'], position: [-0.5],  velocity: [0.0],   effort: [0.0]}"

        joint_command_msg.position = self.position_command.pop(0)

        self.pub_trajectory.publish(joint_command_msg)
    
    def get_joint_pose(self):
        return self.joint_pose
    
    def get_joint_name(self):
        return self.joint_name

    def stop_robot(self):
        # set var to 0
        self.vel_command = []
        self.position_command = []
        self.accel_command = []
        self.command_index = 0
        self.trajectory_progression = 0.0
        self.robot_state = RobotState.STOPPED

        # traj = JointTrajectory()    
        # self.pub_trajectory.publish(traj)
        
    def callback_trajectory_state(self, msg):
        self.trajectory_progression = msg.data 

    def callback_joint_pose(self, joint_pose_msg):
        self.joint_pose = [joint_pose_msg.position[0], 
        joint_pose_msg.position[1], 
        joint_pose_msg.position[2],
        joint_pose_msg.position[3],
        joint_pose_msg.position[8], 
        joint_pose_msg.position[37]] # Stupidities from my isaac implementation...



    def get_progression(self):
        return self.trajectory_progression