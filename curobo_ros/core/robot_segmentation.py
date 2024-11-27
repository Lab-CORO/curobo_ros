import torch
from typing import Tuple

from sensor_msgs.msg import JointState as SensorJointState, PointCloud2, PointField
import rclpy
from rclpy.node import Node

# CuRobo imports
from curobo.types.base import TensorDeviceType
from curobo.types.state import JointState
from curobo.util_file import (
    get_world_configs_path,
    join_path,
    load_yaml,
)
from curobo.wrap.model.robot_world import RobotWorld, RobotWorldConfig
from curobo.geom.types import WorldConfig

import numpy as np

robot_file = "'curobo_doosan/src/m1013/m1013.yml'"

# Create world configuration
world_config = WorldConfig.from_dict(
    load_yaml(join_path(get_world_configs_path(), "collision_wall.yml"))
)

tensor_args = TensorDeviceType()
config = RobotWorldConfig.load_from_config(
    robot_file, world_config, collision_activation_distance=0.0
)
curobo_fn = RobotWorld(config)


class RobotSegmentation(Node):
    """
    Robot segmentation node that subscribes to the fused 3D point cloud
    and masks the robot's points so we can view the work environment without the robot.
    """

    def __init__(
        self,
        robot_world=curobo_fn,
        distance_threshold: float = 0.05,
        ops_dtype: torch.dtype = torch.float32,
    ):
        super().__init__("robot_segmentation")

        self._robot_world = robot_world
        self.distance_threshold = distance_threshold
        self._ops_dtype = ops_dtype
        self.q_js = None
        self.point_cloud = None

        # Create publisher for the masked point cloud
        self.publisher_ = self.create_publisher(PointCloud2, "masked_pointcloud", 10)

        # Subscribe to the fused point cloud
        self.subscription_fused_cloud = self.create_subscription(
            PointCloud2,
            "/fused_pointcloud",
            self.listener_callback_pointcloud,
            10,
        )

        # Subscribe to the robot's joint state
        self.subscription_joint_state = self.create_subscription(
            SensorJointState,
            "/dsr01/joint_states",
            self.listener_callback_jointstate,
            10,
        )

        self.get_logger().info("Segmentation node is initialized")

    # Callback to retrieve the fused point cloud
    def listener_callback_pointcloud(self, msg):
        self.get_logger().info("Point cloud received")
        # Convert PointCloud2 message to numpy array
        cloud_points = self.pointcloud2_to_array(msg)
        if cloud_points is not None:
            self.point_cloud = torch.tensor(cloud_points, dtype=self._ops_dtype)
            # Proceed with segmentation if joint state is available
            if self.q_js is not None:
                self.segment_and_publish()

    # Callback to retrieve the joint state
    def listener_callback_jointstate(self, msg):
        self.get_logger().info("JointState received")
        # Get the current joint state
        self.q_js = JointState(position=torch.tensor(msg.position), joint_names=msg.name)
        # Proceed with segmentation if point cloud is available
        if self.point_cloud is not None:
            self.segment_and_publish()

    def segment_and_publish(self):
        # Perform robot segmentation
        filtered_pointcloud = self._mask_op(self.point_cloud, self.q_js.position)

        # Convert filtered point cloud back to PointCloud2 message
        masked_pc_msg = self.array_to_pointcloud2(filtered_pointcloud)

        # Publish the masked point cloud
        self.publisher_.publish(masked_pc_msg)
        self.get_logger().info("Masked point cloud published")

    def _mask_op(self, point_cloud: torch.Tensor, q: torch.Tensor) -> torch.Tensor:
        if len(q.shape) == 1:
            q = q.unsqueeze(0)

        # Get robot's spheres for the current joint configuration
        robot_spheres = self._robot_world.get_kinematics(q).link_spheres_tensor
        robot_spheres = robot_spheres.view(-1, 4)  # Reshape to (num_spheres, 4)

        # Compute distance from each point to each sphere
        points = point_cloud.unsqueeze(1)  # Shape: (num_points, 1, 3)
        spheres_centers = robot_spheres[:, :3].unsqueeze(0)  # Shape: (1, num_spheres, 3)
        spheres_radii = robot_spheres[:, 3].unsqueeze(0)  # Shape: (1, num_spheres)

        distances = torch.norm(points - spheres_centers, dim=2) - spheres_radii
        min_distances, _ = torch.min(distances, dim=1)

        # Create mask of points that are outside the robot (distance > threshold)
        mask = min_distances > self.distance_threshold

        # Filter the point cloud
        filtered_point_cloud = point_cloud[mask]

        return filtered_point_cloud

    def pointcloud2_to_array(self, cloud_msg):
        """Convert PointCloud2 message to numpy array"""
        # Extract cloud data
        field_names = [field.name for field in cloud_msg.fields]
        if not ('x' in field_names and 'y' in field_names and 'z' in field_names):
            self.get_logger().error("PointCloud2 message does not contain 'x', 'y', 'z' fields.")
            return None

        dtype_list = []
        for field in cloud_msg.fields:
            if field.datatype == PointField.FLOAT32:
                dtype = np.float32
            elif field.datatype == PointField.FLOAT64:
                dtype = np.float64
            else:
                self.get_logger().warn(f"Unsupported PointField datatype: {field.datatype}")
                return None
            dtype_list.append((field.name, dtype))

        cloud_arr = np.frombuffer(cloud_msg.data, dtype=np.dtype(dtype_list))
        points = np.zeros((cloud_msg.width * cloud_msg.height, 3), dtype=np.float32)
        points[:, 0] = cloud_arr['x']
        points[:, 1] = cloud_arr['y']
        points[:, 2] = cloud_arr['z']

        # Remove NaN or infinite values
        mask = np.isfinite(points).all(axis=1)
        points = points[mask, :]

        return points

    def array_to_pointcloud2(self, points: torch.Tensor):
        """Convert numpy array or torch tensor to PointCloud2 message"""
        # Ensure points is a numpy array
        if isinstance(points, torch.Tensor):
            points = points.cpu().numpy()

        # Create PointCloud2 message
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "your_frame_id"  # Set appropriate frame id
        msg.height = 1
        msg.width = points.shape[0]
        msg.is_bigendian = False
        msg.is_dense = True
        msg.point_step = 12  # Size of a point in bytes (3 x 4 bytes for float32)
        msg.row_step = msg.point_step * points.shape[0]

        # Define fields x, y, z
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # Convert points to byte data
        msg.data = np.asarray(points, dtype=np.float32).tobytes()
        return msg


def main(args=None):
    rclpy.init(args=args)
    robot_segmentation = RobotSegmentation()
    rclpy.spin(robot_segmentation)
    robot_segmentation.destroy_node()
    rclpy.shutdown()
