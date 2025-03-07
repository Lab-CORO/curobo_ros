from dsr_msgs2.msg import SpeedjRtStream
from curobo_ros.robot.joint_control_strategy import JointCommandStrategy, RobotState



class DoosanControl(JointCommandStrategy):
    '''
    This class is a strategie to control doosan robot with motion_gen
    '''

    def __init__(self, node, dt):
        # create a publisher
        self.pub_speed_command = node.create_publisher(SpeedjRtStream, '/dsr01/speedj_rt_stream', 10)
        # Creatre a timer at dt
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
            self.robot_state = RobotState.IDLE
        else:
            # create the message with dt
            msg = SpeedjRtStream()
            msg.vel = self.vel_command[self.command_index]
            msg.acc = self.accel_command[self.command_index]
            # msg.time = self.dt
            print(self.vel_command[self.command_index])
            self.command_index += 1
            self.pub_speed_command.publish(msg)
            self.robot_state = RobotState.RUNNING
        return