import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory

# Bibliothèques standard
import time
import os
from typing import Dict, Optional, Tuple, Union

# Bibliothèques tierces
import imageio
import numpy as np
import torch
from nvblox_torch.datasets.mesh_dataset import MeshDataset
import cv2
from cv_bridge import CvBridge

# CuRobo
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
from curobo.geom.types import PointCloud, WorldConfig
from curobo.types.base import TensorDeviceType
from curobo.types.camera import CameraObservation
from curobo.types.math import Pose
from curobo.types.robot import RobotConfig
from curobo.types.state import JointState
from curobo.util_file import get_robot_configs_path, get_world_configs_path, join_path, load_yaml
from curobo.wrap.model.robot_segmenter import RobotSegmenter
from curobo.types.base import TensorDeviceType
from curobo.wrap.model.robot_world import RobotWorld, RobotWorldConfig

robot_file = "'curobo_doosan/src/m1013/m1013.yml'"

# create a world from a dictionary of objects
# cuboid: {} # dictionary of objects that are cuboids
# mesh: {} # dictionary of objects that are meshes
world_config = WorldConfig.from_dict(
            load_yaml(join_path(get_world_configs_path(), "collision_wall.yml")))

tensor_args = TensorDeviceType()
config = RobotWorldConfig.load_from_config(robot_file, world_config,
                                          collision_activation_distance=0.0)
curobo_fn = RobotWorld(config)

class RobotSegmentation(Node):
    '''
    Noeud de segmentation du robot qui a pour objectif de
    s'abonner au nuage de points fusionné des deux caméras et de masquer l'image
    du robot afin que nous puissions voir l'environnement de travail sans le robot.
    '''
    def __init__(
            self,
            robot_world = curobo_fn,
            distance_threshold: float = 0.05,
            use_cuda_graph: bool = True,
            ops_dtype: torch.dtype = torch.float32,
            depth_to_meter: float = 0.001):
        # take from https://github.com/NVlabs/curobo/blob/main/src/curobo/wrap/model/robot_segmenter.py for segmentation

        self._robot_world = robot_world
        self._projection_rays = None
        self.ready = False
        self._out_points_buffer = None
        self._out_gp = None
        self._out_gq = None
        self._out_gpt = None
        self._cu_graph = None
        self._use_cuda_graph = use_cuda_graph
        self.tensor_args = robot_world.tensor_args
        self.distance_threshold = distance_threshold
        self._ops_dtype = ops_dtype
        self._depth_to_meter = depth_to_meter
        super().__init__('robot_segmentation')

   
        

        # Create the publisher for the mask
        self.publisher_ = self.create_publisher(PointCloud2, 'mask_pointcloud', 10)
        # Create subscriber to listen to the fused cloud
        self.subscription_fused_cloud = self.create_subscription(
            PointCloud2,    # Type 
            '/fused_pointcloud',        
            self.listener_callback,
            10)
        self.subscription_fused_cloud  # éviter l'avertissement de variable inutilisée

        # Create subscriber to listen to the joint state of the robot
        self.subscription_joint_state = self.create_subscription(
            JointState,    # Type 
            '/joint_states',        
            self.listener_callback,
            10)
        self.subscription_joint_state  # éviter l'avertissement de variable inutilisée

        self.get_logger().info('Le noeud de segmentation du robot a été initialisé')

    def listener_callback(self, msg):
        self.get_logger().info('Nuage de points fusionné reçu')

        # Préparer l'observation de la caméra
        cam_obs = self.subscription_fused_cloud

        # Obtenir l'état actuel des articulations
        q_js = JointState(position=self.subscription_joint_state.position, joint_names= self.subscription_joint_state.names)

        # Effectuer la segmentation du robot
        depth_mask, filtered_pointcloud = self._mask_op(self,cam_obs,q_js)

        # Publier le nuage de points masqué
        self.publisher_.publish(filtered_pointcloud)
        self.get_logger().info('Nuage de points masqué publié')

        # Publier le nuage de points masqué
        self.publisher_.publish(depth_mask)
        self.get_logger().info('maskage du robot publié')
        

    def _call_op(self, cam_obs, q):
        if self._use_cuda_graph:
            if self._cu_graph is None:
                self._create_cg_graph(cam_obs, q)
            self._cu_cam_obs.copy_(cam_obs)
            self._cu_q.copy_(q)
            self._cu_graph.replay()
            return self._cu_out.clone(), self._cu_filtered_out.clone()
        return self._mask_op(cam_obs, q)

    def _mask_op(self, camera_obs, q):
        if len(q.shape) == 1:
            q = q.unsqueeze(0)

        robot_spheres = self._robot_world.get_kinematics(q).link_spheres_tensor

        points = self.get_pointcloud_from_depth(camera_obs)
        camera_to_robot = camera_obs.pose
        points = points.to(dtype=torch.float32)

        if self._out_points_buffer is None:
            self._out_points_buffer = points.clone()
        if self._out_gpt is None:
            self._out_gpt = torch.zeros((points.shape[0], points.shape[1], 3), device=points.device)
        if self._out_gp is None:
            self._out_gp = torch.zeros((camera_to_robot.position.shape[0], 3), device=points.device)
        if self._out_gq is None:
            self._out_gq = torch.zeros(
                (camera_to_robot.quaternion.shape[0], 4), device=points.device
            )

        points_in_robot_frame = camera_to_robot.batch_transform_points(
            points,
            out_buffer=self._out_points_buffer,
            gp_out=self._out_gp,
            gq_out=self._out_gq,
            gpt_out=self._out_gpt,
        )

        out_points = points_in_robot_frame

        mask, filtered_image = self.mask_spheres_image(
            camera_obs.depth_image, robot_spheres, out_points, self.distance_threshold
        )

        return mask, filtered_image


    def mask_image(
        image: torch.Tensor, distance: torch.Tensor, distance_threshold: float
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        distance = distance.view(
            image.shape[0],
            image.shape[1],
            image.shape[2],
        )
        mask = torch.logical_and((image > 0.0), (distance > -distance_threshold))
        filtered_image = torch.where(mask, 0, image)
        return mask, filtered_image


    def mask_spheres_image(
    image: torch.Tensor,
    link_spheres_tensor: torch.Tensor,
    points: torch.Tensor,
    distance_threshold: float,
    ) -> Tuple[torch.Tensor, torch.Tensor]:

        if link_spheres_tensor.shape[0] != 1:
            assert link_spheres_tensor.shape[0] == points.shape[0]
        if len(points.shape) == 2:
            points = points.unsqueeze(0)

        robot_spheres = link_spheres_tensor.view(link_spheres_tensor.shape[0], -1, 4).contiguous()
        robot_spheres = robot_spheres.unsqueeze(-3)

        robot_radius = robot_spheres[..., 3]
        points = points.unsqueeze(-2)
        sph_distance = -1 * (
            torch.linalg.norm(points - robot_spheres[..., :3], dim=-1) - robot_radius
        )  # b, n_spheres
        distance = torch.max(sph_distance, dim=-1)[0]

        distance = distance.view(
            image.shape[0],
            image.shape[1],
            image.shape[2],
        )
        mask = torch.logical_and((image > 0.0), (distance > -distance_threshold))
        filtered_image = torch.where(mask, 0, image)
        return mask, filtered_image

    def main(args=None):
        rclpy.init(args=args)
        robot_segmentation = RobotSegmentation()
        rclpy.spin(robot_segmentation)
        robot_segmentation.destroy_node()
        rclpy.shutdown()

