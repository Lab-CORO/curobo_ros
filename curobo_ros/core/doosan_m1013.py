from dsr_msgs2.msg import SpeedjRtStream



class DoosanSpeedControl(JointSpeedCommandStrategy):
    '''
    This class is a strategie to control doosan robot with motion_gen
    '''

    def __init__(self, node):
        # create a publisher
        self.pub_speed_command = node.create_publisher(SpeedjRtStream, '/dsr01/speedj_rt_stream', 10)

    def send_command(self, node, vel_command, accel_command, dt):
        # create the message with dt
        msg = SpeedjRtStream()
        msg.vel = vel_command
        msg.acc = accel_command
        self.pub_speed_command(msg)



# ros2 topic pub --rate 100 /dsr01/speedj_rt_stream dsr_msgs2/msg/SpeedjRtStream "{vel:[-10.0, 0.0, 0.0, 0.0, 0.0, 0.0], acc:[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time: 0.20}"
