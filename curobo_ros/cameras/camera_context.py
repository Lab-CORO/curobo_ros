
from curobo_ros.cameras.camera_strategy import CameraStrategy


class CameraContext:
    '''
    The context of the robot if you want to change robot
    '''
    # def __init__(self, camera_strategy: CameraStrategy = None)  :
    camera_strategy:  CameraStrategy

    def set_camera_strategy(self, camera_strategy):
        self.camera_strategy = camera_strategy

    def get_depth_map(self):
        return self.camera_strategy.get_depth_map()

    def get_camera_intrinsic(self):
        return self.camera_strategy.get_camera_intrinsic()



    