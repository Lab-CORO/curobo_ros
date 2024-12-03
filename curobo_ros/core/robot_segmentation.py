import os
import torch
from typing import Tuple

from sensor_msgs.msg import JointState as SensorJointState, PointCloud2, PointField
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

# cuRobo imports
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
from curobo.types.base import TensorDeviceType
from curobo.types.robot import RobotConfig
from curobo.types.state import JointState
from curobo.util_file import (
    get_robot_path,
    get_world_configs_path,
    join_path,
    load_yaml,
)
from curobo.geom.types import WorldConfig

import numpy as np

# Import read_points from sensor_msgs_py
from sensor_msgs_py.point_cloud2 import read_points

# Initialize tensor arguments with CUDA device
tensor_args = TensorDeviceType(device='cuda', dtype=torch.float32)

# Get the path to your curobo_ros package
package_share_directory = get_package_share_directory('curobo_ros')

# Construct the path to m1013.yml within your package
robot_config_file = os.path.join(
    package_share_directory,
    'curobo_doosan', 'src', 'm1013', 'm1013.yml'
)

# Load the robot configuration
config_file = load_yaml(robot_config_file)

# Extract necessary information from the configuration
urdf_file = config_file["robot_cfg"]["kinematics"]["urdf_path"]
base_link = config_file["robot_cfg"]["kinematics"]["base_link"]
ee_link = config_file["robot_cfg"]["kinematics"]["ee_link"]

# Ensure the URDF file path is correct
urdf_file = os.path.join(
    package_share_directory,
    'curobo_doosan', 'src', 'm1013', urdf_file
)

# Generate robot configuration from URDF path, base frame, and end-effector frame
robot_cfg = RobotConfig.from_basic(urdf_file, base_link, ee_link, tensor_args)

# Initialize the CUDA robot model
kin_model = CudaRobotModel(robot_cfg.kinematics)

class RobotSegmentation(Node):
    """
    Robot segmentation node that subscribes to the fused 3D point cloud
    and masks the robot's points so we can view the work environment without the robot.
    """

    def __init__(
        self,
        kin_model=kin_model,
        distance_threshold: float = 0.05,
        ops_dtype: torch.dtype = torch.float32,
    ):
        node_name = 'curobo_robot_segmentation'
        super().__init__(node_name)

        self._kin_model = kin_model
        self.distance_threshold = distance_threshold
        self._ops_dtype = ops_dtype
        self._device = torch.device('cuda')
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

        # Convert PointCloud2 message to numpy array using read_points
        cloud_points = []
        for point in read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            cloud_points.append([point[0], point[1], point[2]])
        cloud_points = np.array(cloud_points, dtype=np.float32)

        if cloud_points.size > 0:
            self.point_cloud = torch.tensor(cloud_points, dtype=self._ops_dtype, device=self._device)
            # Proceed with segmentation if joint state is available
            if self.q_js is not None:
                self.segment_and_publish()
        else:
            self.get_logger().warn("Received empty point cloud.")

    # Callback to retrieve the joint state
    def listener_callback_jointstate(self, msg):
        self.get_logger().info("JointState received")
        # Get the current joint state
        self.q_js = JointState(
            position=torch.tensor(msg.position, dtype=self._ops_dtype, device=self._device),
            joint_names=msg.name
        )
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
        kinematics_state = self._kin_model.get_state(q)
        robot_spheres = kinematics_state.link_spheres_tensor
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

    def array_to_pointcloud2(self, points: torch.Tensor):
        """Convert numpy array or torch tensor to PointCloud2 message"""
        # Ensure points is a numpy array
        if isinstance(points, torch.Tensor):
            points = points.cpu().numpy()

        # Create PointCloud2 message
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"  # Set appropriate frame id
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
        msg.data = points.astype(np.float32).tobytes()
        return msg

def main(args=None):
    rclpy.init(args=args)
    robot_segmentation = RobotSegmentation()
    rclpy.spin(robot_segmentation)
    robot_segmentation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
