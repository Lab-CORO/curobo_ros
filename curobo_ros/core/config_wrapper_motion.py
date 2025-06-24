from .config_wrapper import ConfigWrapper

# Third Party
import torch
from functools import partial

# cuRobo
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
from curobo.types.base import TensorDeviceType
from curobo.geom.sdf.world import CollisionCheckerType, CollisionQueryBuffer
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig
from curobo_msgs.srv import GetCollisionDistance
# ros
import rclpy
from std_srvs.srv import Trigger
from curobo_msgs.srv import Ik


class ConfigWrapperMotion(ConfigWrapper):

    def __init__(self, node, robot):
        super().__init__(node, robot)
        # TODO Have these values be able to be overwritten in a launch file
        # Motion generation parameters
        self.trajopt_tsteps = 32
        self.collision_cache = {"obb": 100} # TODO: make this configurable as ros param
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
            Trigger, node.get_name() + '/update_motion_gen_config', partial(self.set_motion_gen_config, node))
        self.init_services(node)

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
            interpolation_steps = 10000,
            # interpolation_dt=node.get_parameter(
                # 'time_dilation_factor').get_parameter_value().double_value,
            collision_activation_distance=node.get_parameter(
                'collision_activation_distance').get_parameter_value().double_value,
            # acceleration_scale=self.acceleration_scale,
            # TODO issue when modif acceleration scale
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

        self.node_is_available = False
        node_is_available_param = rclpy.parameter.Parameter('node_is_available',rclpy.Parameter.Type.BOOL, False)
        node.set_parameters([node_is_available_param])

        node.motion_gen.warmup()

        node_is_available_param = rclpy.parameter.Parameter('node_is_available',rclpy.Parameter.Type.BOOL, True)
        node.set_parameters([node_is_available_param])

        self.node_is_available = True

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

    def callback_get_collision_distance(self, node, request: GetCollisionDistance, response):
        # get robot spheres poses
        q_js =JointState(position=torch.tensor(self.robot.get_joint_pose(), dtype=self._ops_dtype, device=self._device),
                               joint_names=['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'])
        kinematics_state = self.kin_model.get_state(q_js.position)
        robot_spheres = kinematics_state.link_spheres_tensor.view(1, 1, -1, 4)
        # arg for fct
        tensor_args = TensorDeviceType()
        x_sph = robot_spheres
        query_buffer = CollisionQueryBuffer.initialize_from_shape(
            x_sph.shape, tensor_args, node.motion_gen.world_coll_checker.collision_types
        )
        act_distance = tensor_args.to_device([0.01])
        weight = tensor_args.to_device([1])
        env_query_idx = torch.zeros((x_sph.shape[0]), device=tensor_args.device, dtype=torch.int32)
        sphere_dist = node.world_model.get_sphere_distance(x_sph, query_buffer, weight, act_distance, env_query_idx,
                                                           compute_esdf=True)
        sphere_dist_ar = torch.flatten(sphere_dist, start_dim=0).tolist()
        response.nb_sphere = len(sphere_dist_ar)
        response.data = sphere_dist_ar
        return response


class ConfigWrapperIK(ConfigWrapper):

    def __init__(self, node, robot):
        super().__init__(node, robot)
        self.collision_checker_type = CollisionCheckerType.BLOX
        self.collision_cache = {"obb": 100}  # TODO: make this configurable as ros param

        # Setup service at the end to be sure the node is correctly init befor proposing services. TODO Should use lifecycle
        self.set_ik_gen_config(node, None, None)
        node.ik_init()
        
        self.init_services(node)

        self.motion_gen_srv = node.create_service(
            Trigger, node.get_name() + '/update_motion_gen_config', partial(self.set_ik_gen_config, self))

        self.srv_ik = node.create_service(
            Ik,  node.get_name() +'/ik_batch_poses', node.ik_callback)


    def set_ik_gen_config(self, node, _, response):
        self.world_cfg.blox[0].voxel_size = node.get_parameter(
            'voxel_size').get_parameter_value().double_value

        ik_config = IKSolverConfig.load_from_robot_config(
            self.robot_cfg,
            self.world_cfg,
            rotation_threshold=0.05,
            position_threshold=0.005,
            num_seeds=20,
            self_collision_check=True,
            self_collision_opt=True,
            collision_checker_type=self.collision_checker_type,
            collision_cache=self.collision_cache,
            tensor_args=node.tensor_args,
            use_cuda_graph=False,
        )
        node.ik_solver = IKSolver(ik_config)

        node.world_model = node.ik_solver.world_coll_checker

        # Set the response message when this function is called through the service
        if response is not None:
            response.success = True
            response.message = "Motion generation config set"

        return response

    def update_world_config(self, node):
        node.ik_solver.world_coll_checker.clear_cache()
        node.ik_solver.update_world(self.world_cfg)

    def callback_get_collision_distance(self, node, request: GetCollisionDistance, response):
        # get robot spheres poses
        q_js =JointState(position=torch.tensor(self.robot.get_joint_pose(), dtype=self._ops_dtype, device=self._device),
                               joint_names=['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'])
        kinematics_state = self.kin_model.get_state(q_js.position)
        robot_spheres = kinematics_state.link_spheres_tensor.view(1, 1, -1, 4)
        # arg for fct
        tensor_args = TensorDeviceType()
        x_sph = robot_spheres
        query_buffer = CollisionQueryBuffer.initialize_from_shape(
            x_sph.shape, tensor_args, node.ik_solver.world_coll_checker.collision_types
        )
        act_distance = tensor_args.to_device([0.01])
        weight = tensor_args.to_device([1])
        env_query_idx = torch.zeros((x_sph.shape[0]), device=tensor_args.device, dtype=torch.int32)
        sphere_dist = node.world_model.get_sphere_distance(x_sph, query_buffer, weight, act_distance, env_query_idx,
                                                           compute_esdf=True)
        sphere_dist_ar = torch.flatten(sphere_dist, start_dim=0).tolist()
        response.nb_sphere = len(sphere_dist_ar)
        response.data = sphere_dist_ar
        return response
