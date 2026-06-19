"""
Config wrappers for cuRobo v2 solvers (MotionPlanner, IK, MPC).

v2 notes:
- MotionGen/MpcSolver/IKSolver → MotionPlanner / ModelPredictiveControl / InverseKinematics.
- No more `MotionGenConfig` + `MotionGenPlanConfig` split — solver params live
  on `*Cfg.create(...)`, and per-call params are keyword args of `plan_pose`.
- Collision is integrated into `*Cfg.create()` via `scene_model`, `collision_cache`
  and `self_collision_check`. `CollisionCheckerType` no longer exists.
- `CollisionQueryBuffer` is gone; collision distance goes through the scene
  model directly.
"""

from functools import partial
import torch
import rclpy

from std_srvs.srv import Trigger
from curobo_msgs.srv import GetCollisionDistance, Ik, IkBatch

# v2 runtime flags — must be set before any cuRobo objects are instantiated.
# cuda_graph_reset lets solvers rebuild captured graphs when buffer shapes
# change between plan calls (e.g. different interpolated trajectory lengths).
# Without this, a second plan with a different horizon raises
# "CUDA graph reset is not available." Requires CUDA 12.0+.
# Note: curobo.runtime re-exports (and shadows) curobo._src.runtime values at
# import time — torch_util.is_cuda_graph_reset_available() reads from
# curobo.runtime, so we must flip the flag on the public module too.
import curobo._src.runtime as _curobo_runtime
_curobo_runtime.cuda_graph_reset = True
import curobo.runtime as _curobo_runtime_public
_curobo_runtime_public.cuda_graph_reset = True

from curobo.types import JointState
from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg

from .config_wrapper import ConfigWrapper


class ConfigWrapperMotion(ConfigWrapper):
    """Motion planner config wrapper (v2 `MotionPlanner`)."""

    def __init__(self, node, robot):
        super().__init__(node, robot)

        # v2 trajopt / IK / batch tunables
        self.num_ik_seeds = 32
        self.num_trajopt_seeds = 12
        self.use_cuda_graph = True
        self.interpolation_dt = 0.03
        self.self_collision_check = True
        self.position_tolerance = 0.005
        self.orientation_tolerance = 0.05
        self.max_batch_size = 1
        self.multi_env = False
        self.max_goalset = 1

        self.motion_gen_srv = node.create_service(
            Trigger,
            node.get_name() + '/update_motion_gen_config',
            partial(self.set_motion_gen_config, node),
        )

        self.init_services(node)

    def set_motion_gen_config(self, node, _, response):
        """
        Build (or rebuild) the MotionPlanner and warmup.

        Called at init and on demand via the `update_motion_gen_config` service.
        """
        scene = self.obstacle_manager.get_scene()
        collision_activation_distance = node.get_parameter(
            'collision_activation_distance'
        ).get_parameter_value().double_value

        cfg = MotionPlannerCfg.create(
            robot=self.robot_config_file,
            scene_model=scene,
            num_ik_seeds=self.num_ik_seeds,
            num_trajopt_seeds=self.num_trajopt_seeds,
            position_tolerance=self.position_tolerance,
            orientation_tolerance=self.orientation_tolerance,
            use_cuda_graph=self.use_cuda_graph,
            self_collision_check=self.self_collision_check,
            collision_cache=self.collision_cache,
            optimizer_collision_activation_distance=collision_activation_distance,
            max_batch_size=self.max_batch_size,
            multi_env=self.multi_env,
            max_goalset=self.max_goalset,
        )

        node.motion_planner = MotionPlanner(cfg)
        # Legacy alias — some downstream code still references `node.motion_gen`.
        node.motion_gen = node.motion_planner

        # Output sampling step of the interpolated plan. It's a trajopt config
        # field (not a MotionPlannerCfg.create arg), so set it post-build, before
        # warmup so the interpolation buffer picks it up. Guarded: the standalone
        # node doesn't declare this param.
        if node.has_parameter('interpolation_dt'):
            interp_dt = node.get_parameter('interpolation_dt').get_parameter_value().double_value
            try:
                node.motion_planner.trajopt_solver.config.interpolation_dt = interp_dt
                node.get_logger().info(f"interpolation_dt set to {interp_dt}s")
            except AttributeError:
                node.get_logger().warn("Could not set interpolation_dt on trajopt_solver")

        node.get_logger().info("warming up..")

        self.node_is_available = False
        node.set_parameters([
            rclpy.parameter.Parameter('node_is_available', rclpy.Parameter.Type.BOOL, False)
        ])

        node.motion_planner.warmup()

        node.set_parameters([
            rclpy.parameter.Parameter('node_is_available', rclpy.Parameter.Type.BOOL, True)
        ])
        self.node_is_available = True

        node.get_logger().info("Motion planner configured")

        if response is not None:
            response.success = True
            response.message = "Motion planner config set"
        return response

    def update_world_config(self, node):
        """Push the current Scene into all active solvers."""
        scene = self.obstacle_manager.get_scene()
        if getattr(node, 'motion_planner', None) is not None:
            node.motion_planner.update_world(scene)
        if getattr(node, 'mpc', None) is not None:
            # MPCSolver has no top-level update_world(); go through its checker.
            node.mpc.scene_collision_checker.load_collision_model(scene)

        self.node.get_logger().info(
            f"Updated world: {len(scene.cuboid)} cuboids, {len(scene.mesh)} meshes"
        )

    def callback_get_collision_distance(self, node, request: GetCollisionDistance, response):
        return _compute_sphere_distance(self, node, response)


class ConfigWrapperIK(ConfigWrapper):
    """IK config wrapper (v2 `InverseKinematics`)."""

    def __init__(self, node, robot):
        super().__init__(node, robot)

        self.set_ik_gen_config(node, None, None)
        node.ik_init()

        self.init_services(node)

        self.motion_gen_srv = node.create_service(
            Trigger,
            node.get_name() + '/update_motion_gen_config',
            partial(self.set_ik_gen_config, node),
        )
        self.srv_ik_batch = node.create_service(
            IkBatch, node.get_name() + '/ik_batch_poses', node.ik_batch_callback
        )
        self.srv_ik = node.create_service(
            Ik, node.get_name() + '/ik_pose', node.ik_callback
        )

    def set_ik_gen_config(self, node, _, response):
        scene = self.obstacle_manager.get_scene()

        cfg = InverseKinematicsCfg.create(
            robot=self.robot_config_file,
            scene_model=scene,
            num_seeds=20,
            position_tolerance=0.005,
            orientation_tolerance=0.05,
            self_collision_check=True,
            collision_cache=self.collision_cache,
            use_cuda_graph=False,
        )
        node.ik_solver = InverseKinematics(cfg)

        if response is not None:
            response.success = True
            response.message = "IK solver config set"
        return response

    def update_world_config(self, node):
        node.ik_solver.update_world(self.obstacle_manager.get_scene())

    def callback_get_collision_distance(self, node, request: GetCollisionDistance, response):
        return _compute_sphere_distance(self, node, response)


# ---------------------------------------------------------------------------
# Shared helper

def _compute_sphere_distance(wrapper, node, response):
    """
    Query collision distance for the robot's current configuration.

    v2: the legacy `CollisionQueryBuffer` API is gone. We rely on the solver's
    scene model, which exposes `get_sphere_distance` with a simpler signature.
    """
    q_js = JointState(
        position=torch.tensor(
            wrapper.robot.get_joint_pose(),
            dtype=wrapper._ops_dtype,
            device=wrapper._device,
        ),
        joint_names=wrapper.kin_model.joint_names,
    )
    kinematics_state = wrapper.kin_model.compute_kinematics(q_js)
    # v2: `robot_spheres` replaces `link_spheres_tensor`, shape already [B, H, N, 4]
    robot_spheres = kinematics_state.robot_spheres

    solver = getattr(node, 'motion_planner', None) or getattr(node, 'mpc', None) or getattr(node, 'ik_solver', None)
    if solver is None:
        response.nb_sphere = 0
        response.data = []
        return response

    scene_model = getattr(solver, 'scene_model', None)
    if scene_model is None or not hasattr(scene_model, 'get_sphere_distance'):
        response.nb_sphere = 0
        response.data = []
        return response

    sphere_dist = scene_model.get_sphere_distance(robot_spheres, compute_esdf=True)
    sphere_dist_ar = torch.flatten(sphere_dist, start_dim=0).tolist()
    response.nb_sphere = len(sphere_dist_ar)
    response.data = sphere_dist_ar
    return response
