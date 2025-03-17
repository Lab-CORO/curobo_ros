from curobo_ros.robot.joint_control_strategy import JointCommandStrategy, RobotState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class EmulatorStrategy(JointCommandStrategy):
    '''
    This class is a strategie to control an emulator of the robot with motion_gen
    '''
    
    def __init__(self, node, dt):
        super().__init__(node, dt)
        # create a publisher
        self.pub_speed_command = node.create_publisher(JointState, '/joint_state', 10)
        # Creatre a timer at dt
        self.time_dilation_factor = dt
        self.timer = node.create_timer(dt, self.send_command)
        self.vel_command = []
        self.accel_command = []
        self.command_index = 0
        self.robot_state = RobotState.IDLE
        
    

    def send_command(self):

        if self.command_index >= len(self.vel_command):
            self.robot_state = RobotState.IDLE
            self.command_index = 0
            self.vel_command = []
            self.accel_command = []

        elif(not self.get_send_to_robot()):
            self.robot_state = RobotState.IDLE

        else:
            # create the message with dt
            msg = SpeedjRtStream()
            msg.vel = self.vel_command[self.command_index]
            msg.acc = self.accel_command[self.command_index]
            self.command_index += 1
            self.pub_speed_command.publish(msg)
            self.robot_state = RobotState.RUNNING
        return
    
    def get_joint_pose(self, node):
        ret, joint_pose_msg = wait_for_message(JointState, node, "/dsr01/joint_states", time_to_wait = 0.1)
        if ret:
            return joint_pose_msg.position
        else:
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def stop_robot(self):
        # set var to 0
        self.vel_command = []
        self.accel_command = []
        self.command_index = 0

        self.robot_state = RobotState.STOPPED

        # send msg to robot
        msg = SpeedjRtStream()
        msg.vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.acc = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.time = 0.0
        self.pub_speed_command.publish(msg)
        
    def get_progression(self):
        try:
            percentage = float(self.command_index / len(self.vel_command))
        except:
            percentage = 0.0
        return percentage