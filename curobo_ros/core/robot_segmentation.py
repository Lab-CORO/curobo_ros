import os
import torch
# ros2
from sensor_msgs.msg import PointCloud2, PointField

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
import numpy as np
import open3d as o3d
import ros2_numpy

from curobo_ros.robot.robot_context import RobotContext

# cuRobo imports
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
from curobo.types.base import TensorDeviceType
from curobo.types.robot import RobotConfig
from curobo.types.state import JointState
from curobo.util_file import load_yaml



class RobotSegmentation(Node):
    def __init__(self, distance_threshold=0.05, ops_dtype=torch.float32):
        """
        Initializes the segmentation node, setting up the publishers and subscriptions.
        Also creates the timer callback for real-time segmentation.
        """
        super().__init__('curobo_robot_segmentation')

        # Initialize tensor arguments with CUDA device
        tensor_args = TensorDeviceType(device='cuda', dtype=torch.float32)

        # Get the path to your curobo_ros package
        package_share_directory = get_package_share_directory('curobo_ros')
        self.declare_parameter('robot_config_file',  os.path.join(package_share_directory, 'curobo_doosan', 'src', 'm1013', 'm1013.yml'))
        robot_config_file = self.get_parameter('robot_config_file').get_parameter_value().string_value
        config_file = load_yaml(robot_config_file)
        robot_cfg_dict = config_file["robot_cfg"]
        robot_cfg_dict.pop('cspace', None)
        robot_cfg = RobotConfig.from_dict(robot_cfg_dict, tensor_args)
        urdf_file = robot_cfg.kinematics.generator_config.urdf_path

        if not os.path.isabs(urdf_file):
            urdf_file = os.path.join(package_share_directory, 'curobo_doosan', 'src', 'm1013', urdf_file)
            robot_cfg.kinematics.generator_config.urdf_path = urdf_file

        kin_model = CudaRobotModel(robot_cfg.kinematics)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._kin_model = kin_model
        self.distance_threshold = distance_threshold
        self._ops_dtype = ops_dtype
        self._device = torch.device('cuda')

        self.q_js = JointState(position=torch.tensor([0, 0, 0, 0, 0, 0], dtype=self._ops_dtype, device=self._device),
                                joint_names=[])
        self.point_cloud = None
        self.point_cloud_frame_id = None

        # Robot strategie because of doosan stupidities... 
        self.robot_context = RobotContext(self, 0.03) # dt is not important here.

        self.declare_parameter('joint_states_topic', '/dsr01/joint_states')
        self.declare_parameter('point_cloud_topic', '/fused_pointcloud')
        self.declare_parameter('robot_base_frame', 'base_link')
        joint_states_topic = self.get_parameter('joint_states_topic').get_parameter_value().string_value
        point_cloud_topic = self.get_parameter('point_cloud_topic').get_parameter_value().string_value
        
        # Publisher for the masked point cloud
        self.publisher_ = self.create_publisher(PointCloud2, "masked_pointcloud", 10)
        # Publisher for collision spheres visualization
        self.sphere_marker_pub = self.create_publisher(MarkerArray, 'collision_spheres', 10)

        # Subscription to the fused point cloud topic
        self.subscription_fused_cloud = self.create_subscription(PointCloud2, point_cloud_topic, self.listener_callback_pointcloud, 1)

        # Timer callback for segmentation
        self.create_timer(0.01, self.timer_callback)

        self.get_logger().info("Segmentation node initialized")

    def listener_callback_pointcloud(self, msg):
        """
        Callback for receiving point cloud data.
        Converts the PointCloud2 message into a numpy array and applies filtering using Open3D.
        """
        # self.get_logger().info("Point cloud received")
        cloud_points = ros2_numpy.point_cloud2.pointcloud2_to_array(msg)
        cloud_points = ros2_numpy.point_cloud2.get_xyz_points(cloud_points, remove_nans=True)

        # Convert to Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(cloud_points)

        # Apply voxel down-sampling and remove outliers
        pcd = pcd.voxel_down_sample(voxel_size=0.02)
        pcd, _ = pcd.remove_radius_outlier(nb_points=16, radius=0.05)

        self.point_cloud = torch.tensor(np.asarray(pcd.points), dtype=self._ops_dtype, device=self._device)
        self.point_cloud_frame_id = msg.header.frame_id


    def timer_callback(self):
        """
        Timer callback that triggers the segmentation process every 10 milliseconds.
        Ensures consistent and real-time segmentation updates.
        """
        if self.point_cloud is not None and self.q_js is not None:
            # Get joint state
            self.q_js.position = torch.tensor(self.robot_context.get_joint_pose(), dtype=self._ops_dtype, device=self._device)
            self.q_js.joint_names = self.robot_context.get_joint_name()

            self.segment_and_publish()

    def segment_and_publish(self):
        """
        Performs the segmentation by masking the robot's points in the cloud.
        Publishes the filtered point cloud as a ROS2 message.
        """
        filtered_pointcloud = self._mask_op(self.point_cloud, self.q_js.position)
        masked_pc_msg = self.array_to_pointcloud2(filtered_pointcloud)
        self.publisher_.publish(masked_pc_msg)
        # self.get_logger().info("Masked point cloud published")

    def _mask_op(self, point_cloud: torch.Tensor, q: torch.Tensor) -> torch.Tensor:
        """
        Applies masking operation to remove points belonging to the robot.
        Calculates distances from the cloud points to the robot's collision spheres.
        """
        q = q.unsqueeze(0) if len(q.shape) == 1 else q
        kinematics_state = self._kin_model.get_state(q)
        robot_spheres = kinematics_state.link_spheres_tensor.view(-1, 4)

        points = point_cloud.unsqueeze(1)
        spheres_centers = robot_spheres[:, :3].unsqueeze(0)
        spheres_radii = robot_spheres[:, 3].unsqueeze(0)

        distances = torch.norm(points - spheres_centers, dim=2) - spheres_radii
        min_distances, _ = torch.min(distances, dim=1)
        mask = min_distances > self.distance_threshold
        self.publish_collision_spheres(robot_spheres)
        return point_cloud[mask]

    def array_to_pointcloud2(self, points: torch.Tensor):
        """
        Converts a numpy array or PyTorch tensor into a ROS2 PointCloud2 message.
        """
        if isinstance(points, torch.Tensor):
            points = points.cpu().numpy()

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.point_cloud_frame_id
        msg.height = 1
        msg.width = points.shape[0]
        msg.is_bigendian = False
        msg.is_dense = True
        msg.point_step = 12
        msg.row_step = msg.point_step * points.shape[0]

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = points.astype(np.float32).tobytes()
        return msg

    def publish_collision_spheres(self, robot_spheres):
        """
        Publishes the robot's collision spheres as markers for visualization in RViz.
        Useful for debugging and ensuring proper masking of the robot in the point cloud.
        """
        robot_spheres = robot_spheres.cpu().numpy().tolist()
        marker_array = MarkerArray()
        for i, sphere in enumerate(robot_spheres):
            marker = Marker()
            marker.header.frame_id = self.get_parameter('robot_base_frame').get_parameter_value().string_value
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
    Main entry point for the segmentation node.
    Initializes ROS2, spins the node, and shuts down cleanly on exit.
    """
    rclpy.init(args=args)
    node = RobotSegmentation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()