from abc import ABC, abstractmethod
from std_srvs.srv import SetBool
import threading



class RobotState():
    '''
    This class is a enum to represent the robot state
    Currently useless but maybe later...
    '''
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
        self.send_to_robot = False
        self.buffer_lock = threading.Lock() 
        pass

    def set_command(self, joint_names,  vel_command, accel_command, position_command):
        self.position_command = position_command
        self.vel_command = vel_command
        self.accel_command = accel_command
        self.joint_names = joint_names
        
    @abstractmethod
    def get_joint_pose(self):
        pass 

    @abstractmethod
    def get_joint_name(self):
        pass 
        
    @abstractmethod
    def stop_robot(self):
        pass

    @abstractmethod
    def get_progression(self):
        pass

    @abstractmethod
    def send_trajectrory(self):
        pass

    def get_send_to_robot(self):
        with self.buffer_lock:
            return self.send_to_robot