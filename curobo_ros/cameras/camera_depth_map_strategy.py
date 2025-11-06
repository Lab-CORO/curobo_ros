from curobo_ros.cameras.camera_strategy import CameraStrategy
from curobo.types.camera import CameraObservation
from curobo.types.math import Pose

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from rclpy.wait_for_message import wait_for_message
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
                 frame_id=''
                 ):
        """
        Initialize a depthmap camera strategy.

        Args:
            node: ROS2 node for creating subscriptions and logging
            camera_name: Name of the camera
            topic: Topic name for depth image
            camera_info_topic: Topic name for camera info
            frame_id: Frame ID for the camera
            camera_pose: Camera pose [x, y, z, qw, qx, qy, qz] or None for identity
        """
        super().__init__(node, camera_name, topic, camera_info_topic, frame_id)

        self.depth_map = None
        self.intrinsics = None

        # Wait for camera info message to get intrinsics
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

            node.get_logger().info(f"Camera intrinsics: fx={K[0]:.2f}, fy={K[4]:.2f}, cx={K[2]:.2f}, cy={K[5]:.2f}")
        else:
            node.get_logger().error(f"Failed to receive camera info from {camera_info_topic}")
            raise RuntimeError("Could not get camera intrinsics")



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

      
            # Lookup transform from base_link to camera frame
            t = self.tf_buffer.lookup_transform(
                "base_link",
                self._frame_id,
                rclpy.time.Time())

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

            self.camera_pose = Pose.from_list(camera_pose, tensor_args=self.tensor_args)
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
