#!/usr/bin/env python3

from curobo_ros.cameras.camera_strategy import CameraStrategy
from curobo.types.camera import CameraObservation
from curobo.types.math import Pose

from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from std_msgs.msg import Header
import ros2_numpy
import torch
import numpy as np
import math


class PointCloudCameraStrategy(CameraStrategy):
    """
    Strategy that converts point cloud data to orthographic depth images for cuRobo BLOX.

    This strategy subscribes to a PointCloud2 topic (typically the masked point cloud
    from robot_segmentation) and converts it to an orthographic depth image viewed from above.

    The orthographic projection is ideal for point clouds that are not associated with a
    specific camera viewpoint - it treats the point cloud as a top-down heightmap where
    depth represents the Z-coordinate of points.
    """

    def __init__(self, node, topic='', camera_info=[], frame_id='', pixel_size=0.01, bounds=None):
        """
        Initialize the PointCloud to orthographic depth image strategy.

        Args:
            node: ROS2 node for creating subscriptions and logging
            topic: Topic name for the point cloud subscription
            camera_info: Not used for point cloud (kept for interface compatibility)
            frame_id: Frame ID for the camera
            pixel_size: Size of each pixel in meters (default: 1cm for good resolution)
            bounds: Workspace bounds as [min_x, max_x, min_y, max_y, min_z, max_z]
                   If None, uses default [-1.5, 1.5, -1.5, 1.5, -1.5, 1.5]
        """
        super().__init__(node, topic, camera_info, frame_id)

        # Point cloud data
        # self._obstacle_counter = 0  # Counter for unique obstacle names

        # Orthographic projection parameters
        self.pixel_size = pixel_size
        if bounds is None:
            bounds = [-1.5, 1.5, -1.5, 1.5, -1.5, 1.5]
        self.bounds = bounds

        # Calculate image dimensions from workspace bounds
        self.image_width = int((bounds[1] - bounds[0]) / pixel_size)
        self.image_height = int((bounds[3] - bounds[2]) / pixel_size)

        # Camera pose: positioned above the workspace, looking straight down
        cam_x = (bounds[0] + bounds[1]) / 2
        cam_y = (bounds[2] + bounds[3]) / 2
        cam_z = bounds[5] + 0.5  # 0.5m above the workspace
        
        # Quaternion for looking straight down (rotate -90° around X-axis)
        angle = -math.pi / 2
        qw = math.cos(angle / 2)
        qx = math.sin(angle / 2)
        qy = 0.0
        qz = 0.0

        # TODO Get the camera pose from frame_id
        self.camera_pose = Pose(
                        position=self.tensor_args.to_device([cam_x, cam_y, cam_z]),
                        quaternion=self.tensor_args.to_device([qw, qx, qy, qz]),
                    )
        # Orthographic "intrinsics" (simplified for orthographic projection)
        # fx, fy control the scale: 1 pixel = pixel_size meters
        fx = 1.0 / pixel_size
        fy = 1.0 / pixel_size
        cx = self.image_width / 2.0
        cy = self.image_height / 2.0

        self.intrinsics = torch.tensor([
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0]
        ], dtype=self._dtype, device=self._device)

        # Subscribe to point cloud topic
        self.pointcloud_sub = node.create_subscription(
            PointCloud2,
            self._topic,
            self._update_callback,
            1
        )

        # Publisher for the depth map visualization
        self.depth_map_pub = node.create_publisher(
            Image,
            '/curobo/depth_map',
            10
        )

        # Publisher for the camera info (intrinsics)
        self.camera_info_pub = node.create_publisher(
            CameraInfo,
            '/curobo/depth_map/camera_info',
            10
        )

        node.get_logger().info(
            f"PointCloudCameraStrategy: {self.image_width}x{self.image_height} @ {pixel_size}m/px, "
            f"camera at ({cam_x:.2f}, {cam_y:.2f}, {cam_z:.2f}) looking down"
        )
        node.get_logger().info("Publishing depth map to /curobo/depth_map")
        node.get_logger().info("Publishing camera info to /curobo/depth_map/camera_info")

    def _update_callback(self, msg: PointCloud2):
        """
        Callback for receiving point cloud data.

        Args:
            msg: PointCloud2 message
        """
        try:
            # Convert PointCloud2 to numpy array
            cloud_points = ros2_numpy.point_cloud2.pointcloud2_to_array(msg)
            xyz = ros2_numpy.point_cloud2.get_xyz_points(cloud_points, remove_nans=True)

            # Convert to torch tensor on GPU
            self.pointcloud = torch.tensor(xyz, dtype=self._dtype, device=self._device)
            self.pointcloud_frame_id = msg.header.frame_id
            self.last_update_time = msg.header.stamp

            # Create the depth image
            depth_image = self._create_orthographic_depth_image()

            # Publish the depth map and camera info
            self._publish_depth_map(depth_image)
            self._publish_camera_info()

            # Update curobo world and sync 
            data_camera = CameraObservation(  # rgb_image = data["rgba_nvblox"],
                            depth_image=depth_image , intrinsics=self.intrinsics, pose=self.camera_pose
                        ).to(device=self.tensor_args.device)

            self.node.world_model.add_camera_frame(data_camera, "world")
            self.node.world_model.process_camera_frames('world', False)
            torch.cuda.synchronize()
            self.node.world_model.update_blox_hashes()

        except Exception as e:
            self.node.get_logger().error(f"Error processing point cloud: {e}")

    def _create_orthographic_depth_image(self) -> torch.Tensor:
        """
        Create an orthographic depth image from the point cloud.

        This projects the 3D point cloud onto a 2D grid viewed from above,
        where each pixel's depth value represents the maximum Z-coordinate
        of points falling within that pixel.

        Returns:
            torch.Tensor: Depth image of shape (image_height, image_width)
        """
        # Initialize depth image
        depth_image = torch.zeros(
            (self.image_height, self.image_width),
            dtype=self._dtype,
            device=self._device
        )

        if self.pointcloud is None or self.pointcloud.shape[0] == 0:
            return depth_image

        # Filter points within bounds
        bounds_tensor = torch.tensor(self.bounds, dtype=self._dtype, device=self._device)
        valid_x = (self.pointcloud[:, 0] >= bounds_tensor[0]) & (self.pointcloud[:, 0] < bounds_tensor[1])
        valid_y = (self.pointcloud[:, 1] >= bounds_tensor[2]) & (self.pointcloud[:, 1] < bounds_tensor[3])
        valid_z = (self.pointcloud[:, 2] >= bounds_tensor[4]) & (self.pointcloud[:, 2] < bounds_tensor[5])
        valid_mask = valid_x & valid_y & valid_z

        points_valid = self.pointcloud[valid_mask]

        if points_valid.shape[0] == 0:
            return depth_image

        # Convert world coordinates (x, y) to pixel indices
        # pixel_x = floor((x - min_x) / pixel_size)
        # pixel_y = floor((y - min_y) / pixel_size)
        pixel_x = torch.floor((points_valid[:, 0] - bounds_tensor[0]) / self.pixel_size).long()
        pixel_y = torch.floor((points_valid[:, 1] - bounds_tensor[2]) / self.pixel_size).long()

        # Clamp to image bounds
        pixel_x = torch.clamp(pixel_x, 0, self.image_width - 1)
        pixel_y = torch.clamp(pixel_y, 0, self.image_height - 1)

        # For each pixel, we want the MAXIMUM z value (closest to camera looking down)
        # Calculate distance from camera to each point
        cam_z = self.bounds[5] + 0.5
        depths = cam_z - points_valid[:, 2]  # Distance from camera to point

        # Flatten indices
        flat_indices = pixel_y * self.image_width + pixel_x

        # Use scatter_reduce with 'amin' to keep the minimum depth (closest point)
        depth_image_flat = depth_image.flatten()

        # Initialize with large values for minimum operation
        depth_image_flat.fill_(float('inf'))

        # For each pixel, keep the minimum depth (closest point to camera)
        depth_image_flat.scatter_reduce_(0, flat_indices, depths, reduce='amin', include_self=False)

        # Replace inf with 0 (no data)
        depth_image_flat[depth_image_flat == float('inf')] = 0.0

        depth_image = depth_image_flat.view(self.image_height, self.image_width)

        return depth_image


    def _publish_depth_map(self, depth_image: torch.Tensor):
        """
        Publish the depth map as a ROS Image message for visualization.

        Args:
            depth_image: Depth image tensor (height, width) on GPU
        """
        try:
            # Convert to numpy on CPU
            depth_np = depth_image.cpu().numpy()

            # Normalize for visualization (0-255 range)
            # Non-zero values are mapped to 0-255, zero values stay black
            depth_viz = np.zeros_like(depth_np, dtype=np.uint8)
            mask = depth_np > 0
            if np.any(mask):
                depth_min = depth_np[mask].min()
                depth_max = depth_np[mask].max()
                if depth_max > depth_min:
                    depth_viz[mask] = ((depth_np[mask] - depth_min) / (depth_max - depth_min) * 255).astype(np.uint8)
                else:
                    depth_viz[mask] = 128

            # Create ROS Image message
            msg = Image()
            msg.header = Header()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = self.pointcloud_frame_id if self.pointcloud_frame_id else "base_0"
            msg.height = self.image_height
            msg.width = self.image_width
            msg.encoding = "mono8"
            msg.is_bigendian = False
            msg.step = self.image_width
            msg.data = depth_viz.tobytes()

            # Publish
            self.depth_map_pub.publish(msg)

        except Exception as e:
            self.node.get_logger().error(f"Error publishing depth map: {e}")

    def _publish_camera_info(self):
        """
        Publish the camera intrinsics as a CameraInfo message.
        This provides the calibration parameters for the orthographic depth map.
        """
        try:
            # Create CameraInfo message
            msg = CameraInfo()
            msg.header = Header()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = self.pointcloud_frame_id if hasattr(self, 'pointcloud_frame_id') and self.pointcloud_frame_id else "base_0"

            # Image dimensions
            msg.width = self.image_width
            msg.height = self.image_height

            # Distortion model (none for orthographic projection)
            msg.distortion_model = "plumb_bob"
            msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion

            # Camera intrinsic matrix K (3x3)
            # [fx  0  cx]
            # [ 0 fy  cy]
            # [ 0  0   1]
            intrinsics_np = self.intrinsics.cpu().numpy()
            msg.k = [
                float(intrinsics_np[0, 0]), 0.0, float(intrinsics_np[0, 2]),
                0.0, float(intrinsics_np[1, 1]), float(intrinsics_np[1, 2]),
                0.0, 0.0, 1.0
            ]

            # Rectification matrix R (identity for orthographic)
            msg.r = [
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0
            ]

            # Projection matrix P (3x4)
            # [fx  0  cx  0]
            # [ 0 fy  cy  0]
            # [ 0  0   1  0]
            msg.p = [
                float(intrinsics_np[0, 0]), 0.0, float(intrinsics_np[0, 2]), 0.0,
                0.0, float(intrinsics_np[1, 1]), float(intrinsics_np[1, 2]), 0.0,
                0.0, 0.0, 1.0, 0.0
            ]

            # Binning (no binning)
            msg.binning_x = 0
            msg.binning_y = 0

            # ROI (region of interest - full image)
            msg.roi.x_offset = 0
            msg.roi.y_offset = 0
            msg.roi.height = 0
            msg.roi.width = 0
            msg.roi.do_rectify = False

            # Publish
            self.camera_info_pub.publish(msg)

        except Exception as e:
            self.node.get_logger().error(f"Error publishing camera info: {e}")
