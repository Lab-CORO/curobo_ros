import os
import sys
import torch
from typing import Tuple
#ros2
from sensor_msgs.msg import JointState as SensorJointState, PointCloud2, PointField
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

# cuRobo imports
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
from curobo.types.base import TensorDeviceType
from curobo.types.robot import RobotConfig
from curobo.types.state import JointState
from curobo.util_file import (
    load_yaml,
)

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
robot_cfg_dict = config_file["robot_cfg"]

# Remove 'cspace' from the configuration if it exists
robot_cfg_dict.pop('cspace', None)

# Create the RobotConfig object from the dictionary
robot_cfg = RobotConfig.from_dict(robot_cfg_dict, tensor_args)

# Update the urdf_path to an absolute path if necessary
urdf_file = robot_cfg.kinematics.generator_config.urdf_path
if not os.path.isabs(urdf_file):
    urdf_file = os.path.join(
        package_share_directory,
        'curobo_doosan', 'src', 'm1013', urdf_file
    )
    robot_cfg.kinematics.generator_config.urdf_path = urdf_file

# Initialize the CUDA robot model with the configuration
kin_model = CudaRobotModel(robot_cfg.kinematics)

# Optionally, log the number of collision spheres
# print(f"Number of collision spheres: {len(kin_model.link_spheres)}")

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

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._kin_model = kin_model
        self.distance_threshold = distance_threshold
        self._ops_dtype = ops_dtype
        self._device = torch.device('cuda')
        self.q_js = None
        self.point_cloud = None

        self.sphere_marker_pub = self.create_publisher(MarkerArray, 'collision_spheres', 10)

        # Create publisher for the masked point cloud
        self.publisher_ = self.create_publisher(PointCloud2, "masked_pointcloud", 10)

        # Subscribe to the fused point cloud
        self.subscription_fused_cloud = self.create_subscription(
            PointCloud2,
            "/fused_pointcloud",
            self.listener_callback_pointcloud,
            1,
        )

        # Subscribe to the robot's joint state
        self.subscription_joint_state = self.create_subscription(
            SensorJointState,
            "/dsr01/joint_states",
            self.listener_callback_jointstate,
            1,
        )

        self.get_logger().info("Segmentation node is initialized")

    def listener_callback_pointcloud(self, msg):
        self.get_logger().info("Point cloud received")

        # Convert PointCloud2 message to numpy array using read_points
        cloud_points = []
        for point in read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            cloud_points.append([point[0], point[1], point[2]])
        cloud_points = np.array(cloud_points, dtype=np.float32)



        '''
        RUN pip install open3d
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(cloud_points)
        voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.02)
        cl, ind = pcd.remove_radius_outlier(nb_points=16, radius=0.05)
 
        '''
        # if cloud_points.size > 0:
        #     # Transform the point cloud to the robot's base frame
        #     try:
        #         # Get the transformation from the point cloud frame to the base frame
        #         transform = self.tf_buffer.lookup_transform(
        #             'base_link',  # Target frame
        #             msg.header.frame_id,  # Source frame
        #             rclpy.time.Time(),
        #             timeout=rclpy.duration.Duration(seconds=1.0)
        #         )
        #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #         self.get_logger().warn('Transform not available, cannot transform point cloud')
        #         return

        #     # Apply the transformation to each point
        #     transformed_points = []
        #     for point in cloud_points:
        #         point_stamped = PointStamped()
        #         point_stamped.header.frame_id = msg.header.frame_id
        #         point_stamped.point.x = float(point[0])
        #         point_stamped.point.y = float(point[1])
        #         point_stamped.point.z = float(point[2])
        #         transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
        #         transformed_points.append([
        #             transformed_point.point.x,
        #             transformed_point.point.y,
        #             transformed_point.point.z
        #         ])
        #     cloud_points = np.array(transformed_points, dtype=np.float32)

            # Convert the transformed points to a Torch tensor
        self.point_cloud = torch.tensor(cloud_points, dtype=self._ops_dtype, device=self._device)

        # Print point cloud size for debugging
        self.get_logger().info(f"Point cloud size: {self.point_cloud.shape}")

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

        # Log shapes for debugging
        self.get_logger().info(f"robot_spheres shape: {robot_spheres.shape}")
        self.get_logger().info(f"point_cloud shape: {point_cloud.shape}")

        # Compute distance from each point to each sphere
        points = point_cloud.unsqueeze(1)  # Shape: (num_points, 1, 3)
        spheres_centers = robot_spheres[:, :3].unsqueeze(0)  # Shape: (1, num_spheres, 3)
        spheres_radii = robot_spheres[:, 3].unsqueeze(0)  # Shape: (1, num_spheres)

        self.get_logger().info(f"points shape: {points.shape}")
        self.get_logger().info(f"spheres_centers shape: {spheres_centers.shape}")
        self.get_logger().info(f"spheres_radii shape: {spheres_radii.shape}")

        distances = torch.norm(points - spheres_centers, dim=2) - spheres_radii
        self.get_logger().info(f"distances shape: {distances.shape}")

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

    def publish_collision_spheres(self, robot_spheres):
        marker_array = MarkerArray()
        for i, sphere in enumerate(robot_spheres):
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.id = i
            marker.pose.position.x = sphere[0].item()
            marker.pose.position.y = sphere[1].item()
            marker.pose.position.z = sphere[2].item()
            marker.scale.x = sphere[3].item() * 2  # Diameter
            marker.scale.y = sphere[3].item() * 2
            marker.scale.z = sphere[3].item() * 2
            marker.color.a = 0.5  # Transparency
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)
        self.sphere_marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    robot_segmentation = RobotSegmentation()
    rclpy.spin(robot_segmentation)
    robot_segmentation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
