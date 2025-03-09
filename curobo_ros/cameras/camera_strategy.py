from abc import ABC, abstractmethod

class CameraStrategy:
    '''
    This class is a strategie paterne to implement the joint speed control. 
    It has a timer called every dt to send the next speed command
    '''
    @abstractmethod
    def __init__(self, node):
        pass


    @abstractmethod
    def get_depth_map(self):
        pass


    @abstractmethod
    def get_camera_intrinsic(self):
        pass
        