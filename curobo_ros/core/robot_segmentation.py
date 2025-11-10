import os
import torch
import numpy as np

# ROS2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros

# cuRobo imports
from curobo.types.base import TensorDeviceType
from curobo.types.state import JointState
from curobo.types.camera import CameraObservation
from curobo.types.math import Pose
from curobo.wrap.model.robot_segmenter import RobotSegmenter

from curobo_ros.robot.robot_context import RobotContext


class DepthMapRobotSegmentation(Node):
    def __init__(self, distance_threshold=0.05, ops_dtype=torch.float32):
        """
        Initializes the depth map segmentation node.
        This node removes the robot from depth images using joint states.
        Uses cuRobo's RobotSegmenter for direct depth image masking (no pointcloud conversion).

        Args:
            distance_threshold: Minimum distance (in meters) to consider a point as not part of the robot
            ops_dtype: Data type for tensor operations
        """
        super().__init__('curobo_depth_map_robot_segmentation')

        # Initialize tensor arguments with CUDA device
        self.tensor_args = TensorDeviceType(device='cuda', dtype=ops_dtype)

        # Get the path to your curobo_ros package
        package_share_directory = get_package_share_directory('curobo_ros')
        self.declare_parameter('robot_config_file', os.path.join(
            package_share_directory, 'curobo_doosan', 'src', 'm1013', 'm1013.yml'))
        robot_config_file = self.get_parameter('robot_config_file').get_parameter_value().string_value

        # Extract robot file name (cuRobo expects just the filename with extension)
        # If the path contains the full path, we need to extract just the filename
        robot_file = robot_config_file

        # Initialize TF buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Create RobotSegmenter - this handles all the depth masking logic
        self.get_logger().info(f"Loading robot from: {robot_file}")
        self.curobo_segmenter = RobotSegmenter.from_robot_file(
            robot_file,
            collision_sphere_buffer=0.01,
            distance_threshold=distance_threshold,
            use_cuda_graph=True,
            ops_dtype=ops_dtype,
        )

        self.distance_threshold = distance_threshold
        self._ops_dtype = ops_dtype
        self._device = torch.device('cuda')

        self.depth_image = None
        self.camera_info = None
        self.depth_frame_id = None
        self.bridge = CvBridge()
        self.camera_pose = None  # Camera pose in robot base frame
        self.segmenter_ready = False  # Flag to check if segmenter is ready

        # Robot context for joint states
        self.robot_context = RobotContext(self, 0.03)  # dt is not important here
        self.q_js = None  # Will be initialized when we get joint names

        # Declare parameters
        self.declare_parameter('joint_states_topic', '/dsr01/joint_states')
        self.declare_parameter('depth_image_topic', '/depth_to_rgb/image_raw')
        self.declare_parameter('camera_info_topic', '/depth_to_rgb/camera_info')
        self.declare_parameter('robot_base_frame', 'base_link')

        depth_image_topic = self.get_parameter('depth_image_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value

        # Publisher for the masked depth image
        self.publisher_ = self.create_publisher(Image, "masked_depth_image/image_raw", 10)
        self.publish_camera_info = self.create_publisher(CameraInfo, "masked_depth_image/camera_info", 10)

        # Publisher for collision spheres visualization
        self.sphere_marker_pub = self.create_publisher(MarkerArray, 'collision_spheres', 10)

        # Publisher for robot mask debug (depth image showing only robot)
        self.robot_mask_pub = self.create_publisher(Image, 'robot_mask_debug', 10)

        # Subscription to depth image
        self.subscription_depth = self.create_subscription(
            Image,
            depth_image_topic,
            self.listener_callback_depth,
            1)

        # Subscription to camera info
        self.subscription_camera_info = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.listener_callback_camera_info,
            1)

        # Timer callback for segmentation
        self.create_timer(0.01, self.timer_callback)

        # Timer to log TF frames (debug)
        self.create_timer(2.0, self.log_tf_frames)

        self.get_logger().info("Depth map segmentation node initialized with RobotSegmenter")

    def log_tf_frames(self):
        """
        Logs available TF frames for debugging.
        """
        try:
            # Get all frames
            frames = self.tf_buffer.all_frames_as_yaml()
            self.get_logger().info(f"Available TF frames:\n{frames}", throttle_duration_sec=5.0)
        except Exception as e:
            self.get_logger().warn(f"Could not get TF frames: {e}", throttle_duration_sec=5.0)

    def listener_callback_camera_info(self, msg):
        """
        Callback for receiving camera info data.
        Stores the camera intrinsics matrix.
        """
        self.camera_info = msg

    def listener_callback_depth(self, msg):
        """
        Callback for receiving depth image data.
        Converts the Image message into a tensor.
        """
        try:
            # Convert ROS Image message to numpy array
            # Assuming depth is in millimeters (16UC1) or meters (32FC1)
            if msg.encoding == "16UC1":
                depth_img = self.bridge.imgmsg_to_cv2(msg, "16UC1")
                # Convert from millimeters to meters
                depth_img = depth_img.astype(np.float32) / 1000.0
            elif msg.encoding == "32FC1":
                depth_img = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            else:
                self.get_logger().warn(f"Unsupported depth encoding: {msg.encoding}")
                return

            # Convert to torch tensor
            self.depth_image = torch.from_numpy(depth_img).to(
                dtype=self._ops_dtype, device=self._device)
            self.depth_frame_id = msg.header.frame_id
            
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")

    def get_camera_pose_in_robot_frame(self):
        """
        Gets the camera pose in the robot base frame using TF.

        Returns:
            Pose object or None if transform not available
        """
        try:
            robot_base_frame = self.get_parameter('robot_base_frame').get_parameter_value().string_value

            # Check if the transform is available first
            if not self.tf_buffer.can_transform(
                robot_base_frame,
                self.depth_frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)):
                return None

            # Get transform from robot base to camera frame
            # Use time=0 to get the latest available transform (works with tf_static)
            transform = self.tf_buffer.lookup_transform(
                robot_base_frame,
                self.depth_frame_id,
                rclpy.time.Time())

            # Convert ROS transform to cuRobo Pose
            translation = transform.transform.translation
            rotation = transform.transform.rotation

            # Create 4x4 transformation matrix
            # cuRobo expects [x, y, z, qw, qx, qy, qz]
            pose_list = [
                translation.x, translation.y, translation.z,
                rotation.w, rotation.x, rotation.y, rotation.z
            ]

            pose = Pose.from_list(pose_list, tensor_args=self.tensor_args)
            return pose

        except Exception as e:
            self.get_logger().warn(f"Could not get camera transform: {e}", throttle_duration_sec=1.0)
            return None

    def timer_callback(self):
        """
        Timer callback that triggers the segmentation process every 100 milliseconds.
        """
        if self.depth_image is None or self.camera_info is None:
            return

        # Initialize joint state if needed
        if self.q_js is None:
            joint_names = self.robot_context.get_joint_name()
            if len(joint_names) > 0:
                self.q_js = JointState(
                    position=torch.zeros(len(joint_names), dtype=self._ops_dtype, device=self._device),
                    joint_names=joint_names)

        if self.q_js is None:
            return

        # Get current joint state
        joint_positions = self.robot_context.get_joint_pose()
        if len(joint_positions) != len(self.q_js.joint_names):
            return

        self.q_js.position = torch.tensor(
            joint_positions,
            dtype=self._ops_dtype,
            device=self._device)

        # Get camera pose in robot frame
        camera_pose = self.get_camera_pose_in_robot_frame()
        if camera_pose is None:
            return

        self.segment_and_publish(camera_pose)

    def segment_and_publish(self, camera_pose):
        """
        Performs the segmentation by masking the robot in the depth image.
        Publishes the filtered depth image as a ROS2 message.

        Args:
            camera_pose: Pose object representing camera pose in robot base frame
        """
        # Create CameraObservation from depth image
        # Note: cuRobo expects depth in millimeters, our depth_image is in meters
        depth_mm = self.depth_image * 1000.0

        # Validate depth image
        valid_pixels = torch.sum((depth_mm > 0) & ~torch.isnan(depth_mm) & ~torch.isinf(depth_mm))
        if valid_pixels == 0:
            self.get_logger().warn("Depth image contains no valid pixels", throttle_duration_sec=1.0)
            return

        # Get camera intrinsics as tensor
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        # Validate intrinsics
        if fx <= 0 or fy <= 0:
            self.get_logger().error(f"Invalid camera intrinsics: fx={fx}, fy={fy}")
            return

        # cuRobo expects intrinsics as a 3x3 matrix with batch dimension: (b, 3, 3)
        # Format: [[fx,  0, cx],
        #          [ 0, fy, cy],
        #          [ 0,  0,  1]]
        intrinsics = torch.tensor([
            [[fx, 0.0, cx],
             [0.0, fy, cy],
             [0.0, 0.0, 1.0]]
        ], dtype=self._ops_dtype, device=self._device)

        # Ensure depth image has correct shape: (batch, height, width)
        if depth_mm.dim() == 2:
            depth_mm = depth_mm.unsqueeze(0)  # Add batch dimension (1, H, W)

        # Log shapes for debugging (only once)
        self.get_logger().info(
            f"Depth shape: {depth_mm.shape}, Valid pixels: {valid_pixels}, "
            f"Intrinsics: fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}",
            once=True
        )

        cam_obs = CameraObservation(
            depth_image=depth_mm,
            intrinsics=intrinsics,
            pose=camera_pose
        )

        # Update camera projection if this is the first time
        if not self.curobo_segmenter.ready:
            self.curobo_segmenter.update_camera_projection(cam_obs)

        # Get robot mask from current joint state
        # depth_mask: boolean mask where True = robot pixels, False = world pixels
        # filtered_image: depth image with robot pixels set to 0
        depth_mask, filtered_image = self.curobo_segmenter.get_robot_mask_from_active_js(
            cam_obs,
            self.q_js
        )

        # Publish the filtered depth image (world without robot)
        # Convert from millimeters back to meters and remove batch dimension
        filtered_depth_meters = filtered_image.squeeze(0) / 1000.0
        masked_depth_msg = self.depth_tensor_to_image_msg(filtered_depth_meters)
        self.publisher_.publish(masked_depth_msg)
        self.publish_camera_info.publish(self.camera_info)

        # Publish robot mask debug image if there are subscribers
        if self.robot_mask_pub.get_subscription_count() > 0:
            robot_only = cam_obs.depth_image.clone()
            robot_only[~depth_mask] = 0.0  # Keep only robot pixels
            robot_only_meters = robot_only.squeeze(0) / 1000.0
            robot_mask_msg = self.depth_tensor_to_image_msg(robot_only_meters)
            self.robot_mask_pub.publish(robot_mask_msg)

        # Publish collision spheres for visualization
        if self.sphere_marker_pub.get_subscription_count() > 0:
            self.publish_collision_spheres()


    def depth_tensor_to_image_msg(self, depth_tensor: torch.Tensor) -> Image:
        """
        Converts a depth tensor to a ROS2 Image message.

        Args:
            depth_tensor: torch.Tensor of shape (H, W)

        Returns:
            Image message
        """
        # Convert to numpy and scale back to millimeters for 16UC1 encoding
        depth_np = depth_tensor.cpu().numpy()
        depth_mm = (depth_np * 1000.0).astype(np.uint16)

        # Create Image message
        msg = self.bridge.cv2_to_imgmsg(depth_mm, encoding="16UC1")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.depth_frame_id

        return msg

    def publish_collision_spheres(self):
        """
        Publishes the robot's collision spheres as markers for visualization in RViz.
        Gets the spheres from the current joint state via the robot segmenter.
        """
        if self.q_js is None:
            return

        # Get the robot spheres from the kinematics model in the segmenter
        q = self.q_js.position.unsqueeze(0) if len(self.q_js.position.shape) == 1 else self.q_js.position
        kinematics_state = self.curobo_segmenter._robot_world.kinematics.get_state(q)
        robot_spheres = kinematics_state.link_spheres_tensor.view(-1, 4)

        robot_spheres = robot_spheres.cpu().numpy().tolist()
        marker_array = MarkerArray()

        for i, sphere in enumerate(robot_spheres):
            marker = Marker()
            marker.header.frame_id = self.get_parameter('robot_base_frame').get_parameter_value().string_value
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.id = i
            marker.pose.position.x = sphere[0]
            marker.pose.position.y = sphere[1]
            marker.pose.position.z = sphere[2]
            marker.scale.x = sphere[3] * 2  # Diameter
            marker.scale.y = sphere[3] * 2
            marker.scale.z = sphere[3] * 2
            marker.color.a = 0.5  # Transparency
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)

        self.sphere_marker_pub.publish(marker_array)




def main(args=None):
    """
    Main entry point for the depth map segmentation node.
    """
    rclpy.init(args=args)
    node = DepthMapRobotSegmentation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
