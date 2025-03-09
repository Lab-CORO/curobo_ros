from curobo_ros.cameras.camera_strategy import CameraStrategy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo

import torch
import numpy as np

class RealsenseStrategy(CameraStrategy):
    '''
    This class is a strategie paterne to implement the joint speed control. 
    It has a timer called every dt to send the next speed command
    '''
    def __init__(self, node):

        self.depth_map = None
        self.camera_info = None

        # create sub to info and depthmap
        self.camera_info_sub = node.create_subscription(
            CameraInfo, '/camera/camera/depth/camera_info', self.callback_camera_info, 1)

        self.sub_depth = node.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.callback_depth_map, 1)
                
        # Image processing
        self.bridge = CvBridge()

        
    def get_depth_map(self):  
        return self.depth_map

    def get_camera_intrinsic(self):
        if(self.depth_map is not None):
            self.intrinsics = torch.tensor(
                self.camera_info.k).view(3, 3).float()
        else:
            # retrun a zero matrix 
            self.intrinsics = np.zeros((3, 3), dtype=float)
            self.get_logger().error("Warning: no camera info received")
        return self.intrinsics


    def callback_depth_map(self, msg):
        try:
            depth_img = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            depth_img_float = (depth_img.astype(np.float32))
            self.depth_map = torch.from_numpy(depth_img_float).float()

        except CvBridgeError as e:
            self.get_logger().error(f"An error has occurred: {e}")


    def callback_camera_info(self, msg):
        self.camera_info = msg