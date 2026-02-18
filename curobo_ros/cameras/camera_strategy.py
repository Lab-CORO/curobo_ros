#!/usr/bin/env python3

from abc import ABC, abstractmethod
import torch
import numpy as np
import rclpy
from scipy.spatial.transform import Rotation

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

    def __init__(self, node, camera_name, topic, camera_info, frame_id, intrinsics=None, extrinsics=None):
        """
        Initialize the camera strategy.

        Args:
            node: ROS2 node for creating subscriptions and logging
            camera_name: Name of the camera
            topic: Topic for camera data
            camera_info: Camera info topic or parameters
            frame_id: Frame ID for the camera
            intrinsics: Optional camera intrinsics from config (list or dict)
            extrinsics: Optional camera extrinsics from config (list)
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

    def _parse_intrinsics(self, intrinsics):
        """
        Parse intrinsics from YAML config to 3x3 torch tensor.

        Args:
            intrinsics: Can be:
                - List[float] of length 9: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
                - Dict with keys: fx, fy, cx, cy
                - None: return None (will use camera_info topic)

        Returns:
            torch.Tensor of shape (3, 3) or None
        """
        if intrinsics is None or intrinsics == []:
            return None

        try:
            if isinstance(intrinsics, list):
                if len(intrinsics) != 9:
                    self.node.get_logger().error(
                        f"Intrinsics list must have 9 elements [fx,0,cx,0,fy,cy,0,0,1], got {len(intrinsics)}")
                    return None
                # Reshape [fx, 0, cx, 0, fy, cy, 0, 0, 1] -> [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
                K_matrix = torch.tensor([
                    [intrinsics[0], intrinsics[1], intrinsics[2]],
                    [intrinsics[3], intrinsics[4], intrinsics[5]],
                    [intrinsics[6], intrinsics[7], intrinsics[8]]
                ], dtype=self.tensor_args.dtype, device=self.tensor_args.device)

            elif isinstance(intrinsics, dict):
                required_keys = ['fx', 'fy', 'cx', 'cy']
                if not all(k in intrinsics for k in required_keys):
                    self.node.get_logger().error(
                        f"Intrinsics dict must have keys: {required_keys}")
                    return None
                K_matrix = torch.tensor([
                    [intrinsics['fx'], 0.0, intrinsics['cx']],
                    [0.0, intrinsics['fy'], intrinsics['cy']],
                    [0.0, 0.0, 1.0]
                ], dtype=self.tensor_args.dtype, device=self.tensor_args.device)
            else:
                self.node.get_logger().error(
                    f"Intrinsics must be list or dict, got {type(intrinsics)}")
                return None

            self.node.get_logger().info(
                f"Loaded intrinsics from config: fx={K_matrix[0,0].item():.2f}, "
                f"fy={K_matrix[1,1].item():.2f}, cx={K_matrix[0,2].item():.2f}, cy={K_matrix[1,2].item():.2f}")
            return K_matrix

        except Exception as e:
            self.node.get_logger().error(f"Error parsing intrinsics: {e}")
            return None

    def _parse_extrinsics(self, extrinsics):
        """
        Parse extrinsics from YAML config to cuRobo Pose object.

        Args:
            extrinsics: Can be:
                - List[float] of length 7: [x, y, z, qw, qx, qy, qz]
                - List[List[float]] 4x4 transformation matrix
                - None (will use TF transform)

        Returns:
            Pose object or None
        """
        if extrinsics is None or extrinsics == []:
            return None

        try:
            # Check if it's a 4x4 matrix (list of lists)
            if isinstance(extrinsics, list) and len(extrinsics) == 4 and isinstance(extrinsics[0], list):
                # It's a 4x4 transformation matrix
                matrix = np.array(extrinsics)

                # Extract translation from last column
                position = matrix[:3, 3].tolist()

                # Extract rotation matrix and convert to quaternion
                rotation_matrix = matrix[:3, :3]
                rot = Rotation.from_matrix(rotation_matrix)
                quat_scipy = rot.as_quat()  # Returns [x, y, z, w]

                # cuRobo expects [x, y, z, qw, qx, qy, qz]
                pose_list = position + [quat_scipy[3], quat_scipy[0], quat_scipy[1], quat_scipy[2]]

                camera_pose = Pose.from_list(pose_list, tensor_args=self.tensor_args)

                self.node.get_logger().info(
                    f"Loaded extrinsics from 4x4 matrix: pos=[{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]")
                return camera_pose

            elif isinstance(extrinsics, list) and len(extrinsics) == 7:
                # It's a 7-element list [x, y, z, qw, qx, qy, qz]
                # Validate quaternion norm (should be ~1.0)
                quat = extrinsics[3:]  # [qw, qx, qy, qz]
                quat_norm = sum(q**2 for q in quat) ** 0.5
                if abs(quat_norm - 1.0) > 0.01:
                    self.node.get_logger().warn(
                        f"Quaternion norm is {quat_norm:.4f}, should be 1.0. Normalizing...")
                    quat = [q / quat_norm for q in quat]
                    extrinsics = extrinsics[:3] + quat

                # Create Pose using cuRobo format
                camera_pose = Pose.from_list(extrinsics, tensor_args=self.tensor_args)

                self.node.get_logger().info(
                    f"Loaded extrinsics from config: pos=[{extrinsics[0]:.3f}, {extrinsics[1]:.3f}, {extrinsics[2]:.3f}], "
                    f"quat=[{extrinsics[3]:.3f}, {extrinsics[4]:.3f}, {extrinsics[5]:.3f}, {extrinsics[6]:.3f}]")
                return camera_pose
            else:
                self.node.get_logger().error(
                    f"Extrinsics must be list of 7 elements [x,y,z,qw,qx,qy,qz] or 4x4 matrix, got {type(extrinsics)}")
                return None

        except Exception as e:
            self.node.get_logger().error(f"Error parsing extrinsics: {e}")
            return None



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