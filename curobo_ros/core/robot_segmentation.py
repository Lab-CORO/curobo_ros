import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from ament_index_python.packages import get_package_share_directory

# Bibliothèques standard
import time
import os

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

class RobotSegmentation(Node):
    '''
    Noeud de segmentation du robot qui a pour objectif de
    s'abonner au nuage de points fusionné des deux caméras et de masquer l'image
    du robot afin que nous puissions voir l'environnement de travail sans le robot.
    '''
    def __init__(self):
        super().__init__('robot_segmentation')
        # Create the publisher for the mask
        self.publisher_ = self.create_publisher(PointCloud2, 'mask_pointcloud', 10)
        # Create subscriber to listen to the fused cloud
        self.subscription = self.create_subscription(
            PointCloud2,    # Type 
            '/fused_pointcloud',        
            self.listener_callback,
            10)
        self.subscription  # éviter l'avertissement de variable inutilisée

        # Initialiser le segmentateur de robot take from wrapper_config
        self.robot_file = os.path.join(get_package_share_directory(
            "curobo_ros"), 'curobo_doosan/src/m1013/m1013.yml') 
        self.robot_cfg = load_yaml(self.robot_file)["robot_cfg"]

        self.j_names = self.robot_cfg["kinematics"]["cspace"]["joint_names"]
        self.default_config = self.robot_cfg["kinematics"]["cspace"]["retract_config"]

        self.world_cfg_table = WorldConfig.from_dict(
            load_yaml(join_path(get_world_configs_path(), "collision_wall.yml")))
        self.tensor_args = TensorDeviceType()
        self.curobo_segmenter = RobotSegmenter.from_robot_file(
            self.robot_file, collision_sphere_buffer=0.01, distance_threshold=0.05, use_cuda_graph=True
        )

        self.get_logger().info('Le noeud de segmentation du robot a été initialisé')

    def listener_callback(self, msg):
        self.get_logger().info('Nuage de points fusionné reçu')
        # Convertir le message PointCloud2 en un format utilisable par CuRobo
        pointcloud = self.convert_pointcloud2_to_array(msg)

        # Préparer l'observation de la caméra
        cam_obs = self.create_camera_observation(pointcloud)

        # Obtenir l'état actuel des articulations
        q_js = self.get_current_joint_state()

        # Effectuer la segmentation du robot
        depth_mask, filtered_image = self.curobo_segmenter.get_robot_mask_from_active_js(
            cam_obs,
            q_js,
        )

        # Créer un nouveau message PointCloud2 avec le robot masqué
        masked_pointcloud_msg = self.create_masked_pointcloud_msg(cam_obs, depth_mask)

        # Publier le nuage de points masqué
        self.publisher_.publish(masked_pointcloud_msg)
        self.get_logger().info('Nuage de points masqué publié')

    #pas sur pour cette fonction regarder si vraiment besoin
    def convert_pointcloud2_to_array(self, pointcloud2_msg):
        # Implémenter la conversion de PointCloud2 en un tableau numpy
        pass

    def create_camera_observation(self, pointcloud):
        # Créer un objet CameraObservation à partir du nuage de points
        pass

    # take from : https://github.com/lovelyppp/curobo_seg/blob/main/sdf_reconstruction/robot_image_segmentation_example.py
    # and from : https://github.com/NVlabs/curobo/blob/main/examples/robot_image_segmentation_example.py
    def get_current_joint_state(self):
        # Obtenir l'état actuel des articulations du robot
        kin_model = CudaRobotModel(self.robot_cfg.kinematics)
        q = kin_model.retract_config
        q_js = JointState(position=q, joint_names=kin_model.joint_names)

        return q_js

    def create_masked_pointcloud_msg(self, cam_obs, depth_mask):
        # Créer un nouveau message PointCloud2 à partir du nuage de points masqué
    
        pass

def main(args=None):
    rclpy.init(args=args)
    robot_segmentation = RobotSegmentation()
    rclpy.spin(robot_segmentation)
    robot_segmentation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
