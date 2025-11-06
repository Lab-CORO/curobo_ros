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

    def __init__(self, node, camera_name, topic, camera_info, frame_id):
        """
        Initialize the camera strategy.

        Args:
            node: ROS2 node for creating subscriptions and logging
        """
        self.node = node
        self.name = camera_name
        self._device = torch.device('cuda')
        self._dtype = torch.float32
        self._topic = topic
        self._camera_info = camera_info
        self._frame_id = frame_id
        self.tensor_args = TensorDeviceType()

        # Get camera pose from TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)





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