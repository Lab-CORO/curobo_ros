from .config_wrapper import ConfigWrapper

# Third Party
import torch
from functools import partial
import threading
import time

# cuRobo
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
from curobo.types.base import TensorDeviceType
from curobo.types.robot import JointState
from curobo.geom.sdf.world import CollisionCheckerType, CollisionQueryBuffer
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig
from curobo_msgs.srv import GetCollisionDistance
from curobo.wrap.reacher.mpc import MpcSolver, MpcSolverConfig

# ros
import rclpy
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from curobo_msgs.srv import Ik, IkBatch
from std_msgs.msg import Float32

# Camera management
from curobo_ros.cameras import CameraContext, PointCloudCameraStrategy

class ConfigWrapperMPC(ConfigWrapper):
    def __init__(self, node, robot):
        super().__init__(node, robot)

        # Add a simple ground plane as minimal collision object to satisfy cuRobo's requirements
        from curobo.geom.types import Cuboid
        ground_plane = Cuboid(
            name="ground",
            pose=[0, 0, -0.1, 1, 0, 0, 0],  # 10cm below base
            dims=[3.0, 3.0, 0.01],  # Large thin plane
            color=[0.5, 0.5, 0.5, 1.0]
        )
        self.world_cfg.add_obstacle(ground_plane)

        # Declare MPC parameters
        node.declare_parameter('mpc_step_dt', 0.03)  # Time step for MPC (seconds)
        node.declare_parameter('mpc_horizon_steps', 30)  # Number of steps in MPC horizon

        mpc_step_dt = node.get_parameter('mpc_step_dt').get_parameter_value().double_value
        mpc_horizon_steps = node.get_parameter('mpc_horizon_steps').get_parameter_value().integer_value

        self.mpc_config = MpcSolverConfig.load_from_robot_config(
            self.robot_cfg,
            self.world_cfg,
            store_rollouts=True,
            step_dt=mpc_step_dt,
            horizon=mpc_horizon_steps,
        )
        node.mpc = MpcSolver(self.mpc_config)

        node.get_logger().info(
            f"MPC configured: step_dt={mpc_step_dt}s, horizon={mpc_horizon_steps} steps"
        )


class ConfigWrapperMotion(ConfigWrapper):

    def __init__(self, node, robot):
        super().__init__(node, robot)
        # TODO Have these values be able to be overwritten in a launch file
        # Motion generation parameters
        self.trajopt_tsteps = 32
        # Note: collision_checker_type and collision_cache are now managed by ObstacleManager
        # Default is CollisionCheckerType.BLOX, can be changed via property setter
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


        # Declare parameters for camera configuration
        node.declare_parameter('use_pointcloud_camera', True)
        node.declare_parameter('pointcloud_topic', '/masked_pointcloud')
        node.declare_parameter('pixel_size', 0.01)  # 1cm pixels for orthographic projection

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

        node.get_logger().info("Motion generation config set")

        # Set the response message when this function is called through the service
        if response is not None:
            response.success = True
            response.message = "Motion generation config set"

        return response

    def _restore_obstacles_to_world_checker(self, node):
        """
        Restore all saved obstacles to the world checker after recreation.

        This method is called after creating a new MotionGen to re-add all
        obstacles that were stored in cuboid_list and mesh_list.
        """
        from curobo.geom.types import WorldConfig

        # Build WorldConfig based on current checker type
        if self.current_collision_checker == CollisionCheckerType.MESH:
            # MESH checker: can handle both cuboids and meshes
            world_cfg_to_add = WorldConfig(
                cuboid=self.cuboid_list,
                mesh=self.mesh_list,
            )
            self.node.get_logger().debug(
                f"Restoring to MESH: {len(self.cuboid_list)} cuboids, {len(self.mesh_list)} meshes"
            )
        else:
            # BLOX/PRIMITIVE: convert meshes to cuboids
            combined_cuboids = self.cuboid_list.copy()
            for mesh_obs in self.mesh_list:
                cuboid = mesh_obs.get_cuboid()
                combined_cuboids.append(cuboid)

            world_cfg_to_add = WorldConfig(
                cuboid=combined_cuboids,
                mesh=[],
            )
            self.node.get_logger().debug(
                f"Restoring to BLOX: {len(combined_cuboids)} cuboids (converted {len(self.mesh_list)} meshes)"
            )

        # Update the world_cfg reference
        self.world_cfg = world_cfg_to_add

        # Add obstacles to the motion_gen's world checker
        node.motion_gen.update_world(world_cfg_to_add)

        self.node.get_logger().info(
            f"✅ Restored {len(self.cuboid_list)} cuboids and {len(self.mesh_list)} meshes"
        )

    def recreate_motion_gen(self, node):
        """
        Recreate MotionGen with new collision checker type.

        IMPORTANT: This destroys the current motion_gen and creates a new one.
        All GPU memory is released and reallocated.

        Returns:
            bool: True if recreation successful
        """
        from curobo.wrap.reacher.motion_gen import MotionGenConfig, MotionGen
        from curobo_ros.planners.single_planner import SinglePlanner

        self.node.get_logger().info(
            f"Recreating MotionGen with checker: {self.collision_checker_type}"
        )

        # 0. Temporarily cancel the collision spheres timer to avoid CUDA graph conflicts
        # During CUDA graph capture (warmup), no other CUDA operations can occur
        timer_was_active = False
        if hasattr(self, 'publish_collision_spheres_timer') and self.publish_collision_spheres_timer is not None:
            self.node.get_logger().info("Pausing collision spheres timer during MotionGen recreation...")
            self.publish_collision_spheres_timer.cancel()
            timer_was_active = True

        # 1. Clear current motion_gen
        if hasattr(node, 'motion_gen') and node.motion_gen is not None:
            node.motion_gen.world_coll_checker.clear_cache()
            del node.motion_gen
            node.motion_gen = None

        # 2. Update collision cache
        # Note: This delegates to ObstacleManager._update_collision_cache()
        # Ensures cache is synchronized before recreating MotionGen
        self._update_collision_cache()

        # 3. Recreate MotionGen with new checker type
        try:
            # Use the same parameters as in set_motion_gen_config()
            motion_gen_config = MotionGenConfig.load_from_robot_config(
                self.robot_cfg,
                self.world_cfg,
                node.tensor_args,
                trajopt_tsteps=self.trajopt_tsteps,
                collision_cache=self.collision_cache,
                collision_checker_type=self.collision_checker_type,
                use_cuda_graph=self.use_cuda_graph,
                num_trajopt_seeds=self.num_trajopt_seeds,
                num_graph_seeds=self.num_graph_seeds,
                interpolation_steps=10000,
                collision_activation_distance=node.get_parameter(
                    'collision_activation_distance').get_parameter_value().double_value,
                self_collision_check=self.self_collision_check,
                maximum_trajectory_dt=self.maximum_trajectory_dt,
            )

            # Create new MotionGen
            node.motion_gen = MotionGen(motion_gen_config)

            # Warmup (important!)
            # CRITICAL: No other CUDA operations can occur during warmup (CUDA graph capture)
            self.node.get_logger().info("Warming up new MotionGen...")
            node.motion_gen.warmup(
                enable_graph=True,
                warmup_js_trajopt=False,
            )

            # Update shared instance for all planners
            SinglePlanner.set_motion_gen(node.motion_gen)

            # Update current checker
            self.current_collision_checker = self.collision_checker_type

            # Re-add all saved obstacles to the new world checker
            self.node.get_logger().info(
                f"Re-adding {len(self.cuboid_list)} cuboids and {len(self.mesh_list)} meshes to new checker..."
            )
            self._restore_obstacles_to_world_checker(node)

            # Restore collision spheres timer if it was active
            if timer_was_active:
                from functools import partial
                self.node.get_logger().info("Resuming collision spheres timer...")
                self.publish_collision_spheres_timer = self.node.create_timer(
                    0.5, partial(self.publish_collision_spheres, self.node)
                )

            self.node.get_logger().info(
                f"MotionGen recreated successfully with {self.collision_checker_type}"
            )

            return True

        except Exception as e:
            self.node.get_logger().error(f"Failed to recreate MotionGen: {e}")
            import traceback
            self.node.get_logger().error(traceback.format_exc())

            # Restore timer even on failure
            if timer_was_active:
                from functools import partial
                self.node.get_logger().info("Restoring collision spheres timer after error...")
                self.publish_collision_spheres_timer = self.node.create_timer(
                    0.5, partial(self.publish_collision_spheres, self.node)
                )

            return False

    def update_world_config(self, node):
        """
        Update world configuration based on current collision checker.

        This is called when obstacles are added/removed manually (not during checker switch).
        """
        # Clear cache if safe to do so
        # For BLOX checker, _blox_mapper is only created after first update_world() call
        if hasattr(node.motion_gen.world_coll_checker, '_blox_mapper'):
            if node.motion_gen.world_coll_checker._blox_mapper is not None:
                node.motion_gen.world_coll_checker.clear_cache()
        else:
            # For MESH/PRIMITIVE checkers, clear_cache is always safe
            node.motion_gen.world_coll_checker.clear_cache()

        # Restore obstacles using the shared method
        self._restore_obstacles_to_world_checker(node)

        self.node.get_logger().info(
            f"Updated world: {len(self.cuboid_list)} cuboids, {len(self.mesh_list)} meshes "
            f"(checker: {self.current_collision_checker})"
        )

    

    def callback_get_collision_distance(self, node, request: GetCollisionDistance, response):
        # get robot spheres poses
        q_js =JointState(position=torch.tensor(self.robot.get_joint_pose(), dtype=self._ops_dtype, device=self._device),
                               joint_names=self.robot.get_joint_name())
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

        self.srv_ik_batch = node.create_service(
            IkBatch,  node.get_name() +'/ik_batch_poses', node.ik_batch_callback)

        self.srv_ik = node.create_service(
            Ik,  node.get_name() +'/ik_pose', node.ik_callback)


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
                               joint_names=self.robot.get_joint_name())
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
