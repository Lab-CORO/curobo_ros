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
# ros
import rclpy
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from curobo_msgs.srv import Ik, IkBatch
from std_msgs.msg import Float32


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

        # Warmup progress tracking
        self.warmup_progress = 0.0  # Percentage 0.0 to 1.0
        self.warmup_complete = threading.Event()
        self.warmup_lock = threading.Lock()

        # Declare ROS parameter for warmup progress
        node.declare_parameter('warmup_progress', 0.0)

        # Create publisher for warmup progress
        self.warmup_progress_pub = node.create_publisher(
            Float32,
            node.get_name() + '/warmup_progress',
            10
        )

        self.motion_gen_srv = node.create_service(
            Trigger, node.get_name() + '/update_motion_gen_config', partial(self.set_motion_gen_config, node))
        self.init_services(node)

    def set_motion_gen_config(self, node, _, response):
        '''
        This function sets the motion generation configuration for the trajectory generation node.
        It launches the warmup in a background thread to avoid blocking the main thread.
        '''
        # Create a thread for the warmup
        warmup_thread = threading.Thread(
            target=self._warmup_worker,
            args=(node,),
            daemon=True,
            name="curobo_warmup_thread"
        )

        node.get_logger().info("Starting warmup in background thread...")
        warmup_thread.start()

        # If this is a service call, return immediately
        if response is not None:
            response.success = True
            response.message = "Warmup started in background. Monitor /warmup_progress topic for status."
            return response

        return None

    def _warmup_worker(self, node):
        '''
        Function executed in a separate thread to perform the warmup with progress tracking
        '''
        try:
            with self.warmup_lock:
                # Estimate the warmup duration
                estimated_warmup_time = self._estimate_warmup_duration()

                node.get_logger().info(
                    f"Warmup thread: Starting warmup (estimated duration: {estimated_warmup_time:.1f}s)"
                )

                # Mark node as unavailable
                self._update_node_availability(node, False)

                # Update world configuration
                self.world_cfg.blox[0].voxel_size = node.get_parameter(
                    'voxel_size').get_parameter_value().double_value

                # Create motion generation config
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
                    finetune_dt_scale=self.finetune_dt_scale,
                    fixed_iters_trajopt=self.fixed_iters_trajopt,
                    finetune_trajopt_iters=self.finetune_trajopt_iters,
                    minimize_jerk=self.minimize_jerk,
                )

                # Create motion generator
                node.motion_gen = MotionGen(motion_gen_config)

                # Launch progress monitoring thread
                progress_thread = threading.Thread(
                    target=self._monitor_warmup_progress,
                    args=(node, estimated_warmup_time),
                    daemon=True,
                    name="warmup_progress_monitor"
                )
                progress_thread.start()

                node.get_logger().info("Warmup thread: Starting CUDA warmup (this may take 30-60 seconds)...")

                # WARMUP - blocking but in separate thread
                start_time = time.time()
                node.motion_gen.warmup()
                actual_duration = time.time() - start_time

                # Update to 100%
                self.warmup_progress = 1.0
                self._update_warmup_param(node, 1.0)
                self._publish_warmup_progress(1.0)

                node.get_logger().info(
                    f"✅ Warmup complete in {actual_duration:.1f}s (estimated: {estimated_warmup_time:.1f}s)"
                )

                # Finalize
                node.world_model = node.motion_gen.world_collision
                self._update_node_availability(node, True)
                self.warmup_complete.set()

                node.get_logger().info("✅ Motion generation ready to use!")

        except Exception as e:
            node.get_logger().error(f"❌ Warmup failed: {str(e)}")
            import traceback
            node.get_logger().error(traceback.format_exc())
            self.warmup_progress = 0.0
            self._update_warmup_param(node, 0.0)
            self._publish_warmup_progress(0.0)
            self.warmup_complete.set()

    def _monitor_warmup_progress(self, node, estimated_duration):
        '''
        Thread that monitors and publishes warmup progress
        '''
        start_time = time.time()
        update_interval = 0.5  # Update every 0.5 seconds
        last_log_time = 0

        while self.warmup_progress < 1.0:
            elapsed = time.time() - start_time
            progress = min(elapsed / estimated_duration, 0.99)

            # S-curve for more natural progress
            # First 40% of time = 20% progress (CUDA compilation)
            # Next 50% of time = 60% progress (tensor warmup)
            # Last 10% of time = 20% progress (finalization)
            if progress < 0.4:
                self.warmup_progress = progress * 0.2 / 0.4  # 0-20%
            elif progress < 0.9:
                self.warmup_progress = 0.2 + (progress - 0.4) * 0.6 / 0.5  # 20-80%
            else:
                self.warmup_progress = 0.8 + (progress - 0.9) * 0.2 / 0.1  # 80-99%

            # Update ROS parameter and publish
            self._update_warmup_param(node, self.warmup_progress)
            self._publish_warmup_progress(self.warmup_progress)

            # Log periodically (every 5 seconds)
            if int(elapsed) % 5 == 0 and int(elapsed) != last_log_time and elapsed > 0:
                node.get_logger().info(
                    f"Warmup progress: {self.warmup_progress*100:.1f}% ({elapsed:.1f}s / {estimated_duration:.1f}s)"
                )
                last_log_time = int(elapsed)

            time.sleep(update_interval)

    def _estimate_warmup_duration(self):
        '''
        Estimate warmup duration based on configuration
        '''
        base_time = 10.0  # Base time (loading, init)

        if self.use_cuda_graph:
            base_time += 25.0  # CUDA graph compilation

        # Time proportional to number of seeds
        base_time += self.num_trajopt_seeds * 1.5
        base_time += self.num_graph_seeds * 0.8

        # Time proportional to timesteps
        base_time += (self.trajopt_tsteps / 10) * 0.3

        # Adjustment for collision checker type
        if self.collision_checker_type == CollisionCheckerType.BLOX:
            base_time += 5.0

        return base_time

    def _update_warmup_param(self, node, progress):
        '''
        Update ROS parameter for warmup progress
        '''
        try:
            warmup_param = Parameter(
                'warmup_progress',
                Parameter.Type.DOUBLE,
                float(progress)
            )
            node.set_parameters([warmup_param])
        except Exception:
            pass  # Ignore parameter update errors

    def _publish_warmup_progress(self, progress):
        '''
        Publish warmup progress to ROS topic
        '''
        try:
            msg = Float32()
            msg.data = float(progress)
            self.warmup_progress_pub.publish(msg)
        except Exception:
            pass  # Ignore publish errors

    def _update_node_availability(self, node, available: bool):
        '''
        Update node availability status
        '''
        self.node_is_available = available
        node_is_available_param = Parameter(
            'node_is_available',
            Parameter.Type.BOOL,
            available
        )
        node.set_parameters([node_is_available_param])

    def wait_for_warmup(self, timeout=None):
        '''
        Blocking function to wait for warmup to complete

        Args:
            timeout: Maximum wait time in seconds (None = infinite)

        Returns:
            True if warmup is complete, False if timeout
        '''
        return self.warmup_complete.wait(timeout=timeout)

    def update_world_config(self, node):
        node.motion_gen.world_coll_checker.clear_cache()
        node.motion_gen.update_world(self.world_cfg)

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
