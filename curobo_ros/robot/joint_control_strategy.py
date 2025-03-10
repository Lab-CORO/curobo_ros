from abc import ABC, abstractmethod


class RobotState():
    RUNNING = 0
    STOPPED = 1
    READY = 2
    ERROR = 3
    IDLE = 4

class JointCommandStrategy:
    '''
    This class is a strategie paterne to implement the joint speed control. 
    It has a timer called every dt to send the next speed command
    '''
    @abstractmethod
    def __init__(self, node, dt):
        pass

    def set_command(self, joint_names,  vel_command, accel_command, position_command):
        self.position_command = position_command
        self.vel_command = vel_command
        self.accel_command = accel_command
        self.joint_names = joint_names
        
        

    @abstractmethod
    def send_command(self, node, vel_command, accel_command, dt):
        pass

    @abstractmethod
    def get_joint_pose(self, node):
        pass 

