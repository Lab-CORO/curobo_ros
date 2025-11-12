#!/usr/bin/env python3

from abc import ABC, abstractmethod
import torch
import numpy as np
import rclpy

from curobo.types.camera import CameraObservation
from curobo.types.math import Pose
from curobo.types.base import TensorDeviceType

from tf2_ros import Buffer, TransformListener, TransformException
import ros2_numpy


class CameraStrategy(ABC):
    """
    Abstract base class for camera strategies.
    Each strategy implements how to obtain camera observations for cuRobo collision checking.
    """

    def __init__(self, node, camera_name, topic, camera_info, frame_id, extrinsic_matrix=None, intrinsic_matrix=None):
        """
        Initialize the camera strategy.

        Args:
            node: ROS2 node for creating subscriptions and logging
            camera_name: Name of the camera
            topic: ROS topic for camera data
            camera_info: Camera info topic or path
            frame_id: TF frame ID for the camera (optional if extrinsic_matrix provided)
            extrinsic_matrix: Camera extrinsic matrix [4x4] as fallback for frame_id
            intrinsic_matrix: Camera intrinsic matrix [3x3] as fallback for camera_info
        """
        self.node = node
        self.name = camera_name
        self._device = torch.device('cuda')
        self._dtype = torch.float32
        self._topic = topic
        self._camera_info = camera_info
        self._frame_id = frame_id
        self._extrinsic_matrix = extrinsic_matrix
        self._intrinsic_matrix = intrinsic_matrix
        self.tensor_args = TensorDeviceType()

        # Get camera pose from TF if frame_id is available
        if self._frame_id:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, node)
        else:
            self.tf_buffer = None
            self.tf_listener = None
            if not self._extrinsic_matrix:
                node.get_logger().warn(
                    f"Camera '{camera_name}': No frame_id and no extrinsic_matrix provided. "
                    "Camera pose lookup will fail."
                )





    def set_update_callback(self, callback):
        """
        Set a callback to be called when new camera data is received.

        Args:
            callback: Function to call when new data is received
        """
        self._update_callback = callback

    def _update_callback(self, msg):
        """
        Update the camera from the new image/pcd incommings.
        """
        pass