from curobo_ros.cameras.camera_strategy import CameraStrategy
from curobo.types.camera import CameraObservation
from curobo.types.math import Pose

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo

import torch
import numpy as np


class RealsenseStrategy(CameraStrategy):
    '''
    Camera strategy for Intel RealSense depth cameras.

    This strategy subscribes to depth image and camera info topics to provide
    camera observations for cuRobo collision checking.
    '''
    def __init__(self, node, depth_topic='/camera/depth/image_rect_raw',
                 camera_info_topic='/camera/depth/camera_info',
                 camera_pose=None):
        """
        Initialize the RealSense camera strategy.

        Args:
            node: ROS2 node for creating subscriptions and logging
            depth_topic: Topic name for depth image
            camera_info_topic: Topic name for camera info
            camera_pose: Camera pose [x, y, z, qw, qx, qy, qz] or None for identity
        """
        super().__init__(node)

        self.depth_map = None
        self.camera_info = None

        # Camera pose (default is identity if not provided)
        if camera_pose is None:
            camera_pose = [0, 0, 0, 1, 0, 0, 0]  # Identity pose
        self.camera_pose = Pose.from_list(camera_pose)

        # Create subscriptions to info and depth map
        self.camera_info_sub = node.create_subscription(
            CameraInfo, camera_info_topic, self.callback_camera_info, 1)

        self.sub_depth = node.create_subscription(
            Image, depth_topic, self.callback_depth_map, 1)

        # Image processing
        self.bridge = CvBridge()

        node.get_logger().info(f"RealsenseStrategy initialized with depth topic: {depth_topic}")

    def get_camera_observation(self) -> CameraObservation:
        """
        Get camera observation for cuRobo.

        Returns:
            CameraObservation: Contains depth_image, intrinsics, and pose
        """
        if not self.is_ready():
            self.node.get_logger().warn("Camera observation requested but camera not ready")
            return None

        # Get intrinsics as tensor
        intrinsics = self._get_intrinsics_tensor()

        # Ensure depth map is on correct device
        depth_map = self.depth_map.to(device=self._device, dtype=self._dtype)

        # Create and return camera observation
        observation = CameraObservation(
            depth_image=depth_map,
            intrinsics=intrinsics,
            pose=self.camera_pose
        )

        return observation

    def _get_intrinsics_tensor(self) -> torch.Tensor:
        """
        Get camera intrinsics as a torch tensor.

        Returns:
            torch.Tensor: 3x3 intrinsics matrix
        """
        if self.camera_info is not None:
            intrinsics = torch.tensor(
                self.camera_info.k,
                dtype=self._dtype,
                device=self._device
            ).view(3, 3)
        else:
            # Return identity matrix if no camera info
            intrinsics = torch.eye(3, dtype=self._dtype, device=self._device)
            self.node.get_logger().warn("Warning: no camera info received, using identity matrix")

        return intrinsics

    def is_ready(self) -> bool:
        """
        Check if the camera has received both depth and intrinsics data.

        Returns:
            bool: True if camera is ready
        """
        return self.depth_map is not None and self.camera_info is not None

    def get_camera_pose(self) -> Pose:
        """
        Get the camera pose in the world frame.

        Returns:
            Pose: Camera pose
        """
        return self.camera_pose

    def set_camera_pose(self, pose):
        """
        Update the camera pose.

        Args:
            pose: Camera pose as Pose object or list [x, y, z, qw, qx, qy, qz]
        """
        if isinstance(pose, list):
            self.camera_pose = Pose.from_list(pose)
        else:
            self.camera_pose = pose

    def callback_depth_map(self, msg):
        """
        Callback for receiving depth image data.

        Args:
            msg: Image message
        """
        try:
            depth_img = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            depth_img_float = depth_img.astype(np.float32)
            self.depth_map = torch.from_numpy(depth_img_float)

        except CvBridgeError as e:
            self.node.get_logger().error(f"An error has occurred: {e}")

    def callback_camera_info(self, msg):
        """
        Callback for receiving camera info data.

        Args:
            msg: CameraInfo message
        """
        self.camera_info = msg