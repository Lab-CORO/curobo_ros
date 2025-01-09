import os
from curobo_msgs.srv import AddObject, RemoveObject, GetVoxelGrid
from geometry_msgs.msg import Point32, Vector3

from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.types import WorldConfig, Cuboid, Capsule, Cylinder, Sphere, Mesh, VoxelGrid
from curobo.util_file import load_yaml, join_path, get_world_configs_path
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig

from ament_index_python.packages import get_package_share_directory
import ros2_numpy as rnp
import numpy as np
import torch
from .config_wrapper import ConfigWrapper




import time

import rclpy
import std_msgs.msg
from sensor_msgs.msg import JointState
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_srvs.srv import Trigger
import numpy as np

# Third Party
import torch

# cuRobo
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import RobotConfig
from curobo.util_file import get_robot_configs_path, join_path, load_yaml, get_world_configs_path
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig

from curobo.geom.types import WorldConfig
from curobo.geom.sdf.world import CollisionCheckerType
# msg ik
from curobo_msgs.srv import Ik

from functools import partial

import rclpy
from rclpy.node import Node
from .wait_for_message import wait_for_message

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState as SensorJointState
from sensor_msgs.msg import Image, CameraInfo
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from std_srvs.srv import Trigger
from curobo_msgs.srv import AddObject, Fk, RemoveObject, GetVoxelGrid
# from .config_wrapper import ConfigWrapper
# from .config_wrapper_motion import ConfigWrapperMotion

from curobo.geom.types import Cuboid
from curobo.types.base import TensorDeviceType
from curobo.types.camera import CameraObservation
from curobo.types.math import Pose
from curobo.types.robot import JointState
from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig
from .marker_publisher import MarkerPublisher



class ConfigWrapperMotion(ConfigWrapper):

    def __init__(self, node):
        super().__init__(node)
        # TODO Have these values be able to be overwritten in a launch file
        # Motion generation parameters
        self.trajopt_tsteps = 32
        self.collision_cache = {"obb": 10}
        self.collision_checker_type = CollisionCheckerType.BLOX
        self.use_cuda_graph = True
        self.num_trajopt_seeds = 12
        self.num_graph_seeds = 12
        self.interpolation_dt = 0.03
        self.acceleration_scale = 1.0
        self.self_collision_check = True
        self.maximum_trajectory_dt = 0.25
        self.finetune_dt_scale = 1.05
        self.fixed_iters_trajopt = True
        self.finetune_trajopt_iters = 300
        self.minimize_jerk = True

        self.motion_gen_srv = node.create_service(
            Trigger, node.get_name() + '/update_motion_gen_config', partial(self.set_motion_gen_config, self))


    def set_motion_gen_config(self, node, _, response):
        '''
        This function sets the motion generation configuration for the trajectory generation node.
        It is called by the service callback created in the node.
        It takes the node as an argument to access its parameters and set its motion generation configuration.
        The MotionGenConfig class is set through a warmup function that uses the updated value.

        The function is also used at the node's initialization to set the motion generation configuration.
        In that case, the response argument is not used.
        '''
        # Update the world configuration
        self.world_cfg.blox[0].voxel_size = node.get_parameter(
            'voxel_size').get_parameter_value().double_value

        # Set the motion generation configuration with the values stored in this wrapper and the node
        motion_gen_config = MotionGenConfig.load_from_robot_config(
            self.robot_cfg,
            self.world_cfg,
            node.tensor_args,
            trajopt_tsteps=self.trajopt_tsteps,
            # TODO Test with different values
            collision_cache=self.collision_cache,
            collision_checker_type=self.collision_checker_type,
            use_cuda_graph=self.use_cuda_graph,
            num_trajopt_seeds=self.num_trajopt_seeds,
            num_graph_seeds=self.num_graph_seeds,
            interpolation_dt=self.interpolation_dt,
            collision_activation_distance=node.get_parameter(
                'collision_activation_distance').get_parameter_value().double_value,
            acceleration_scale=self.acceleration_scale,
            self_collision_check=self.self_collision_check,
            maximum_trajectory_dt=self.maximum_trajectory_dt,
            finetune_dt_scale=self.finetune_dt_scale,
            fixed_iters_trajopt=self.fixed_iters_trajopt,
            finetune_trajopt_iters=self.finetune_trajopt_iters,
            minimize_jerk=self.minimize_jerk,
        )

        # Set the motion generation configuration in the node and warmup the motion generation
        node.motion_gen = MotionGen(motion_gen_config)

        node.get_logger().info("warming up..")

        node.motion_gen.warmup()

        node.world_model = node.motion_gen.world_collision

        # TODO Remove this when RViz has the ability to visualize the objects itself
        node.debug_voxel()

        node.get_logger().info("Motion generation config set")

        # Set the response message when this function is called through the service
        if response is not None:
            response.success = True
            response.message = "Motion generation config set"

        return response

    def update_world_config(self, node):
        node.motion_gen.world_coll_checker.clear_cache()
        node.motion_gen.update_world(self.world_cfg)
        node.debug_voxel()


class ConfigWrapperIK(ConfigWrapper):

    def __init__(self, node):
        super().__init__(node)
        self.collision_checker_type = CollisionCheckerType.BLOX

        self.motion_gen_srv = node.create_service(
            Trigger, node.get_name() + '/update_motion_gen_config', partial(self.set_ik_gen_config, self))

    def set_ik_gen_config(self, node, _, response):
        ik_config = IKSolverConfig.load_from_robot_config(
            self.robot_cfg,
            self.world_cfg,
            rotation_threshold=0.05,
            position_threshold=0.005,
            num_seeds=20,
            self_collision_check=True,
            self_collision_opt=True,
            collision_checker_type=self.collision_checker_type,
            tensor_args=node.tensor_args,
            use_cuda_graph=True,
        )
        node.ik_solver = IKSolver(ik_config)
       
        # Set the response message when this function is called through the service
        if response is not None:
            response.success = True
            response.message = "Motion generation config set"

        return response
    
    def update_world_config(self, node):
        node.ik_solver.world_coll_checker.clear_cache()
        node.ik_solver.update_world(self.world_cfg)
        
        