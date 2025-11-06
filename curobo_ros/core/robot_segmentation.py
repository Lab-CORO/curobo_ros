import os
import torch
import numpy as np

# ROS2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header
from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import struct

# cuRobo imports
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
from curobo.types.base import TensorDeviceType
from curobo.types.robot import RobotConfig
from curobo.types.state import JointState
from curobo.util_file import load_yaml

from curobo_ros.robot.robot_context import RobotContext


class DepthMapRobotSegmentation(Node):
    def __init__(self, distance_threshold=0.05, ops_dtype=torch.float32):
        """
        Initializes the depth map segmentation node.
        This node removes the robot from depth images using joint states.

        Args:
            distance_threshold: Minimum distance (in meters) to consider a point as not part of the robot
            ops_dtype: Data type for tensor operations
        """
        super().__init__('curobo_depth_map_robot_segmentation')

        # Initialize tensor arguments with CUDA device
        tensor_args = TensorDeviceType(device='cuda', dtype=torch.float32)

        # Get the path to your curobo_ros package
        package_share_directory = get_package_share_directory('curobo_ros')
        self.declare_parameter('robot_config_file', os.path.join(
            package_share_directory, 'curobo_doosan', 'src', 'm1013', 'm1013.yml'))
        robot_config_file = self.get_parameter('robot_config_file').get_parameter_value().string_value
        config_file = load_yaml(robot_config_file)
        robot_cfg_dict = config_file["robot_cfg"]
        robot_cfg_dict.pop('cspace', None)
        robot_cfg = RobotConfig.from_dict(robot_cfg_dict, tensor_args)
        urdf_file = robot_cfg.kinematics.generator_config.urdf_path


        kin_model = CudaRobotModel(robot_cfg.kinematics)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._kin_model = kin_model
        self.distance_threshold = distance_threshold
        self._ops_dtype = ops_dtype
        self._device = torch.device('cuda')

        self.q_js = JointState(
            position=torch.tensor([0, 0, 0, 0, 0, 0], dtype=self._ops_dtype, device=self._device),
            joint_names=[])

        self.depth_image = None
        self.camera_info = None
        self.depth_frame_id = None
        self.bridge = CvBridge()

        # Robot context for joint states
        self.robot_context = RobotContext(self, 0.03)  # dt is not important here

        # Declare parameters
        self.declare_parameter('joint_states_topic', '/dsr01/joint_states')
        self.declare_parameter('depth_image_topic', '/depth_to_rgb/image_raw')
        self.declare_parameter('camera_info_topic', '/depth_to_rgb/camera_info')
        self.declare_parameter('robot_base_frame', 'base_link')

        depth_image_topic = self.get_parameter('depth_image_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value

        # Publisher for the masked depth image
        self.publisher_ = self.create_publisher(Image, "masked_depth_image", 10)

        # Publisher for collision spheres visualization
        self.sphere_marker_pub = self.create_publisher(MarkerArray, 'collision_spheres', 10)

        # Publisher for robot point cloud (debug)
        self.robot_pointcloud_pub = self.create_publisher(PointCloud2, 'robot_pointcloud_debug', 10)

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

        self.get_logger().info("Depth map segmentation node initialized")

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

    def timer_callback(self):
        """
        Timer callback that triggers the segmentation process every 10 milliseconds.
        """
        if self.depth_image is not None and self.camera_info is not None and self.q_js is not None:
            # Get joint state
            self.q_js.position = torch.tensor(
                self.robot_context.get_joint_pose(),
                dtype=self._ops_dtype,
                device=self._device)
            self.q_js.joint_names = self.robot_context.get_joint_name()

            self.segment_and_publish()

    def segment_and_publish(self):
        """
        Performs the segmentation by masking the robot in the depth image.
        Publishes the filtered depth image as a ROS2 message.
        """
        masked_depth = self._mask_depth_image(self.depth_image, self.q_js.position)
        masked_depth_msg = self.depth_tensor_to_image_msg(masked_depth)
        self.publisher_.publish(masked_depth_msg)

    def depth_to_pointcloud(self, depth_image: torch.Tensor, camera_intrinsics: dict) -> torch.Tensor:
        """
        Converts a depth image to a 3D point cloud.

        Args:
            depth_image: torch.Tensor of shape (H, W) with depth values in meters
            camera_intrinsics: dict with keys 'fx', 'fy', 'cx', 'cy'

        Returns:
            torch.Tensor of shape (N, 3) representing 3D points (x, y, z)
        """
        H, W = depth_image.shape

        # Get camera intrinsics
        fx = camera_intrinsics['fx']
        fy = camera_intrinsics['fy']
        cx = camera_intrinsics['cx']
        cy = camera_intrinsics['cy']

        # Create pixel coordinate grid
        v_coords, u_coords = torch.meshgrid(
            torch.arange(H, dtype=self._ops_dtype, device=self._device),
            torch.arange(W, dtype=self._ops_dtype, device=self._device),
            indexing='ij')

        # Get valid depth values (non-zero and non-nan)
        valid_mask = (depth_image > 0) & (~torch.isnan(depth_image)) & (~torch.isinf(depth_image))

        # Extract valid pixels
        u_valid = u_coords[valid_mask]
        v_valid = v_coords[valid_mask]
        z_valid = depth_image[valid_mask]

        # Convert to 3D coordinates using pinhole camera model
        # x = (u - cx) * z / fx
        # y = (v - cy) * z / fy
        x = (u_valid - cx) * z_valid / fx
        y = (v_valid - cy) * z_valid / fy
        z = z_valid

        # Stack into (N, 3) point cloud
        points = torch.stack([x, y, z], dim=1)

        return points, valid_mask

    def pointcloud_to_depth(self, points: torch.Tensor, camera_intrinsics: dict,
                           image_shape: tuple) -> torch.Tensor:
        """
        Converts a 3D point cloud back to a depth image.

        Args:
            points: torch.Tensor of shape (N, 3) representing 3D points
            camera_intrinsics: dict with keys 'fx', 'fy', 'cx', 'cy'
            image_shape: tuple (H, W) for the output depth image

        Returns:
            torch.Tensor of shape (H, W) representing the depth image
        """
        H, W = image_shape

        # Get camera intrinsics
        fx = camera_intrinsics['fx']
        fy = camera_intrinsics['fy']
        cx = camera_intrinsics['cx']
        cy = camera_intrinsics['cy']

        # Initialize depth image with zeros
        depth_image = torch.zeros((H, W), dtype=self._ops_dtype, device=self._device)

        # Project 3D points to 2D pixel coordinates
        x, y, z = points[:, 0], points[:, 1], points[:, 2]

        # Avoid division by zero
        valid_z = z > 0
        x_valid = x[valid_z]
        y_valid = y[valid_z]
        z_valid = z[valid_z]

        # Project to pixel coordinates
        u = (x_valid * fx / z_valid + cx).long()
        v = (y_valid * fy / z_valid + cy).long()

        # Filter points within image bounds
        valid_pixels = (u >= 0) & (u < W) & (v >= 0) & (v < H)
        u_valid = u[valid_pixels]
        v_valid = v[valid_pixels]
        z_final = z_valid[valid_pixels]

        # Fill depth image (handle occlusions by keeping closest depth)
        # Vectorized approach: sort points by depth (far to near) and scatter
        # Points written last (nearest) will overwrite farther points at same pixel

        if len(z_final) > 0:
            # Sort by depth (descending order: farthest to nearest)
            # This way, when we scatter, nearest points overwrite farther ones
            sorted_indices = torch.argsort(z_final, descending=True)
            u_sorted = u_valid[sorted_indices]
            v_sorted = v_valid[sorted_indices]
            z_sorted = z_final[sorted_indices]

            # Convert 2D indices (v, u) to 1D linear indices
            linear_indices = v_sorted * W + u_sorted

            # Scatter depth values into flattened depth image
            # Later values (closer points) will overwrite earlier ones (farther points)
            depth_image.view(-1).scatter_(0, linear_indices, z_sorted)

        return depth_image

    def _mask_depth_image(self, depth_image: torch.Tensor, q: torch.Tensor) -> torch.Tensor:
        """
        Masks the depth image by removing robot points.

        Strategy:
        1. Convert depth image to point cloud
        2. Apply robot masking on point cloud (reusing existing logic)
        3. Convert masked point cloud back to depth image

        Args:
            depth_image: torch.Tensor of shape (H, W)
            q: torch.Tensor of joint positions

        Returns:
            torch.Tensor of shape (H, W) with robot pixels masked (set to 0)
        """
        # Get camera intrinsics from camera_info
        camera_intrinsics = {
            'fx': self.camera_info.k[0],
            'fy': self.camera_info.k[4],
            'cx': self.camera_info.k[2],
            'cy': self.camera_info.k[5]
        }

        # Step 1: Convert depth image to point cloud
        points, valid_mask = self.depth_to_pointcloud(depth_image, camera_intrinsics)

        # Step 2: Apply robot masking (reuse logic from point cloud segmentation)
        filtered_points = self._mask_pointcloud(points, q)

        # Step 3: Convert filtered point cloud back to depth image
        masked_depth = self.pointcloud_to_depth(
            filtered_points,
            camera_intrinsics,
            depth_image.shape)

        return masked_depth

    def _mask_pointcloud(self, point_cloud: torch.Tensor, q: torch.Tensor) -> torch.Tensor:
        """
        Applies masking operation to remove points belonging to the robot.
        This is the same logic as in the original robot_segmentation.py

        Args:
            point_cloud: torch.Tensor of shape (N, 3)
            q: torch.Tensor of joint positions

        Returns:
            torch.Tensor of filtered points (M, 3) where M <= N
        """
        q = q.unsqueeze(0) if len(q.shape) == 1 else q
        kinematics_state = self._kin_model.get_state(q)
        robot_spheres = kinematics_state.link_spheres_tensor.view(-1, 4)

        points = point_cloud.unsqueeze(1)  # (N, 1, 3)
        spheres_centers = robot_spheres[:, :3].unsqueeze(0)  # (1, S, 3)
        spheres_radii = robot_spheres[:, 3].unsqueeze(0)  # (1, S)

        # Calculate distances from each point to each sphere
        distances = torch.norm(points - spheres_centers, dim=2) - spheres_radii
        min_distances, _ = torch.min(distances, dim=1)

        # Keep points that are farther than threshold
        mask = min_distances > self.distance_threshold

        # Inverted mask: points that are part of the robot (masked out)
        robot_mask = ~mask

        # Publish collision spheres for visualization
        self.publish_collision_spheres(robot_spheres)

        # Publish robot point cloud for debug (actual masked points, not sphere centers)
        if self.robot_pointcloud_pub.get_subscription_count() > 0:
            masked_points = point_cloud[mask]  # Points that belong to the robot
            if masked_points.shape[0] > 0:  # Only publish if there are masked points
                timestamp = self.get_clock().now().to_msg()
                robot_frame = self.get_parameter('robot_base_frame').get_parameter_value().string_value
                pc_msg = self._create_pointcloud2_msg(masked_points, robot_frame, timestamp)
                self.robot_pointcloud_pub.publish(pc_msg)

        return point_cloud[mask]

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

    def publish_collision_spheres(self, robot_spheres):
        """
        Publishes the robot's collision spheres as markers for visualization in RViz.

        Args:
            robot_spheres: torch.Tensor of shape (N, 4) where each row is [x, y, z, radius]
        """
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


    def _create_pointcloud2_msg(self, points: torch.Tensor, frame_id: str, timestamp) -> PointCloud2:
        """
        Create a PointCloud2 message from a torch tensor of 3D points.

        Args:
            points: torch.Tensor of shape (N, 3) containing XYZ coordinates
            frame_id: Frame ID for the point cloud
            timestamp: ROS timestamp

        Returns:
            PointCloud2 message
        """
        # Convert tensor to numpy on CPU
        points_np = points.cpu().numpy()

        # Create PointCloud2 message
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = timestamp
        msg.header.frame_id = frame_id

        # Define point cloud fields (X, Y, Z)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        msg.is_bigendian = False
        msg.point_step = 12  # 3 * 4 bytes (float32)
        msg.row_step = msg.point_step * points_np.shape[0]
        msg.is_dense = True
        msg.height = 1
        msg.width = points_np.shape[0]

        # Pack point data
        buffer = []
        for point in points_np:
            buffer.append(struct.pack('fff', point[0], point[1], point[2]))

        msg.data = b''.join(buffer)

        return msg


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
