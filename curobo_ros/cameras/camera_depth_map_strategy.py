from curobo_ros.cameras.camera_strategy import CameraStrategy
from curobo.types.camera import CameraObservation
from curobo.types.math import Pose

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from rclpy.wait_for_message import wait_for_message
from tf2_ros import TransformException
import rclpy
from scipy.spatial.transform import Rotation

import torch
import numpy as np


class DepthMapCameraStrategy(CameraStrategy):
    '''
    Camera strategy for Intel RealSense depth cameras.

    This strategy subscribes to depth image and camera info topics to provide
    camera observations for cuRobo collision checking.
    '''
    def __init__(self, node, camera_name, topic='/camera/depth/image_rect_raw',
                 camera_info_topic='/camera/depth/camera_info',
                 frame_id='',
                 extrinsic_matrix=None,
                 intrinsic_matrix=None
                 ):
        """
        Initialize a depthmap camera strategy.

        Args:
            node: ROS2 node for creating subscriptions and logging
            camera_name: Name of the camera
            topic: Topic name for depth image
            camera_info_topic: Topic name for camera info
            frame_id: Frame ID for the camera (optional if extrinsic_matrix provided)
            extrinsic_matrix: Camera extrinsic matrix [4x4] as fallback for TF
            intrinsic_matrix: Camera intrinsic matrix [3x3] as fallback for camera_info
        """
        super().__init__(node, camera_name, topic, camera_info_topic, frame_id,
                         extrinsic_matrix=extrinsic_matrix, intrinsic_matrix=intrinsic_matrix)

        self.depth_map = None
        self.intrinsics = None

        # Try to get intrinsics from provided matrix first, then fallback to ROS topic
        if self._intrinsic_matrix is not None:
            # Use provided intrinsic matrix
            try:
                if isinstance(self._intrinsic_matrix, (list, np.ndarray)):
                    intrinsic_array = np.array(self._intrinsic_matrix, dtype=np.float32)
                    if intrinsic_array.shape != (3, 3):
                        node.get_logger().error(
                            f"Invalid intrinsic_matrix shape: {intrinsic_array.shape}. Expected (3, 3)"
                        )
                        raise ValueError("intrinsic_matrix must be 3x3")

                    self.intrinsics = torch.from_numpy(intrinsic_array).to(
                        dtype=self.tensor_args.dtype, device=self.tensor_args.device
                    )
                    node.get_logger().info(
                        f"Using provided intrinsic_matrix: fx={intrinsic_array[0,0]:.2f}, "
                        f"fy={intrinsic_array[1,1]:.2f}, cx={intrinsic_array[0,2]:.2f}, "
                        f"cy={intrinsic_array[1,2]:.2f}"
                    )
            except Exception as e:
                node.get_logger().error(f"Failed to parse intrinsic_matrix: {e}")
                self.intrinsics = None

        # Fallback to camera info topic if intrinsics not loaded
        if self.intrinsics is None:
            node.get_logger().info(f"Waiting for camera info on {camera_info_topic}...")
            res, camera_info_msg = wait_for_message(CameraInfo, node, camera_info_topic, time_to_wait=5.0)

            if res:
                # Extract intrinsics from camera_info and create 3x3 matrix
                # K is a flattened 3x3 matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
                K = camera_info_msg.k

                # Create intrinsics as a 3x3 matrix (shape: (3, 3))
                # CameraObservation will add batch dimension automatically if needed
                self.intrinsics = torch.tensor([
                    [K[0], K[1], K[2]],  # [fx,  0, cx]
                    [K[3], K[4], K[5]],  # [ 0, fy, cy]
                    [K[6], K[7], K[8]]   # [ 0,  0,  1]
                ], dtype=self.tensor_args.dtype, device=self.tensor_args.device)

                node.get_logger().info(f"Camera intrinsics from ROS: fx={K[0]:.2f}, fy={K[4]:.2f}, cx={K[2]:.2f}, cy={K[5]:.2f}")
            else:
                node.get_logger().error(f"Failed to receive camera info from {camera_info_topic}")
                raise RuntimeError("Could not get camera intrinsics from topic or intrinsic_matrix")



        # Create subscription to depth topic
        self.sub_depth = self.node.create_subscription(
            Image, topic, self.callback_depth_map, 1)

        # Image processing
        self.bridge = CvBridge()

        node.get_logger().info(f"DepthMap camera initialized with depth topic: {topic}")


    def callback_depth_map(self, msg):
        """
        Callback for receiving depth image data.

        Args:
            msg: Image message
        """
        try:
            # Convert ROS image to numpy array
            # Assume depth is in millimeters (16UC1) or meters (32FC1)
            if msg.encoding == "16UC1":
                depth_img = self.bridge.imgmsg_to_cv2(msg, "16UC1")
                # Convert millimeters to meters
                depth_img_float = depth_img.astype(np.float32) / 1000.0
            elif msg.encoding == "32FC1":
                depth_img_float = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            else:
                self.node.get_logger().warn(f"Unsupported depth encoding: {msg.encoding}, trying 32FC1")
                depth_img_float = self.bridge.imgmsg_to_cv2(msg, "32FC1")

            # Convert to torch tensor and move to GPU
            depth_tensor = torch.from_numpy(depth_img_float).to(
                device=self.tensor_args.device,
                dtype=self.tensor_args.dtype
            )

            # Get camera pose from TF or extrinsic matrix
            self.camera_pose = self._get_camera_pose()

            if self.camera_pose is None:
                self.node.get_logger().error("Failed to get camera pose from TF or extrinsic_matrix")
                return
            # Create a camera observation (similar to realsense_collision.py line 231-233)
            data_camera = CameraObservation(
                depth_image=depth_tensor,
                intrinsics=self.intrinsics,
                pose=self.camera_pose
            )

            # Add camera frame to world model if available
            if hasattr(self.node, 'world_model') and self.node.world_model is not None:
                self.node.world_model.add_camera_frame(data_camera, "world")
                self.node.world_model.process_camera_frames("world", False)
                torch.cuda.synchronize()
                self.node.world_model.update_blox_hashes()

            self.depth_map = depth_tensor

        except CvBridgeError as e:
            self.node.get_logger().error(f"CvBridge error: {e}")
        except Exception as e:
            self.node.get_logger().error(f"Error processing depth image: {e}")
        except TransformException as ex:
            # Fallback to identity pose if transform not available
            self.node.get_logger().warn(
                f'Could not transform base_link to {self._frame_id}: {ex}. Using identity pose.')

    def _get_camera_pose(self) -> Pose:
        """
        Get camera pose from TF (preferred) or extrinsic matrix (fallback).

        Returns:
            Pose object or None if both TF and extrinsic_matrix are unavailable
        """
        # Try TF lookup first if frame_id is available
        if self._frame_id and self.tf_buffer is not None:
            try:
                t = self.tf_buffer.lookup_transform(
                    "base_link",
                    self._frame_id,
                    rclpy.time.Time()
                )

                # Extract translation
                translation = t.transform.translation
                position = [translation.x, translation.y, translation.z]

                # Extract rotation and convert to quaternion using scipy
                rotation_msg = t.transform.rotation
                # ROS quaternion format: (x, y, z, w)
                quat_ros = [rotation_msg.x, rotation_msg.y, rotation_msg.z, rotation_msg.w]

                # Convert to scipy Rotation
                rot = Rotation.from_quat(quat_ros)  # scipy expects [x, y, z, w]
                quat_scipy = rot.as_quat()  # Returns [x, y, z, w]

                # cuRobo expects [x, y, z, qw, qx, qy, qz]
                camera_pose = position + [quat_scipy[3], quat_scipy[0], quat_scipy[1], quat_scipy[2]]

                return Pose.from_list(camera_pose, tensor_args=self.tensor_args)

            except TransformException as ex:
                self.node.get_logger().warn(
                    f'Could not get transform from base_link to {self._frame_id}: {ex}. '
                    'Trying extrinsic_matrix...'
                )

        # Fallback to extrinsic matrix if TF failed or unavailable
        if self._extrinsic_matrix is not None:
            try:
                extrinsic_array = np.array(self._extrinsic_matrix, dtype=np.float32)
                if extrinsic_array.shape != (4, 4):
                    self.node.get_logger().error(
                        f"Invalid extrinsic_matrix shape: {extrinsic_array.shape}. Expected (4, 4)"
                    )
                    return None

                # Extract translation from 4x4 matrix
                position = extrinsic_array[:3, 3].tolist()

                # Extract rotation from 4x4 matrix and convert to quaternion
                rotation_matrix = extrinsic_array[:3, :3]
                rot = Rotation.from_matrix(rotation_matrix)
                quat_scipy = rot.as_quat()  # Returns [x, y, z, w]

                # cuRobo expects [x, y, z, qw, qx, qy, qz]
                camera_pose = position + [quat_scipy[3], quat_scipy[0], quat_scipy[1], quat_scipy[2]]

                # self.node.get_logger().info(f"Using extrinsic_matrix for camera pose")
                return Pose.from_list(camera_pose, tensor_args=self.tensor_args)

            except Exception as e:
                self.node.get_logger().error(f"Failed to parse extrinsic_matrix: {e}")
                return None

        # If neither TF nor extrinsic_matrix available, return identity pose
        self.node.get_logger().warn(
            "No frame_id or extrinsic_matrix available. Using identity pose."
        )
        return Pose(
            position=self.tensor_args.to_device([0, 0, 0]),
            quaternion=self.tensor_args.to_device([1, 0, 0, 0]),
        )
