from curobo_ros.robot.ghost_strategy import GhostStrategy
from curobo_ros.robot.joint_control_strategy import JointCommandStrategy, RobotState


class RobotContext:
    '''
    The context of the robot if you want to change robot
    '''
    # def __init__(self, robot_strategy: JointSpeedCommandStrategy = None)  :
    robot_strategy: JointCommandStrategy

    def set_robot_strategy(self, robot_strategy, node, dt):
        self.robot_strategy = robot_strategy
        self.ghost_strategy = GhostStrategy(node, dt) 

    # def send_speed_command(self, vel_command, accel_command, dt):
    #     self.robot_strategy.send_command(node,  vel_command, accel_command, dt)

    def set_command(self, joint_names,  vel_command, accel_command, position_command):
        # If there is a real robot 
        self.robot_strategy.set_command(joint_names, vel_command, accel_command, position_command)
        # Active all the time for rviz visualisation
        self.ghost_strategy.set_command(joint_names, vel_command, accel_command, position_command)
        self.ghost_strategy.send_command()

    def get_joint_pose(self, node):
        return self.robot_strategy.get_joint_pose(node)

    