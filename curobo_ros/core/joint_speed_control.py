from abc import ABC, abstractmethod

class JointSpeedCommandStrategy:
    '''
    This class is a strategie paterne to implement the joint speed control. 
    It has a timer called every dt to send the next speed command
    '''
    @abstractmethod
    def __init__(self, node):
        pass

    @abstractmethod
    def send_command(self, node, vel_command, accel_command, dt):
        pass


class RobotContext:
    '''
    The context of the robot if you want to change robot
    '''
    def __init__(self, robot_strategy: JointSpeedCommandStrategy = None)  :
        self.robot_strategy = robot_strategy

    def set_robot_strategy(self, robot_strategy):
        self.robot_strategy = robot_strategy

    def send_speed_command(self, node,  vel_command, accel_command, dt):
        self.robot_strategy.send_command(node,  vel_command, accel_command, dt)


    