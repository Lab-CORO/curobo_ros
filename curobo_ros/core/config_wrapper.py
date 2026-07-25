import os
from abc import ABC, abstractmethod

# Import all manager classes
from curobo_ros.core.config_manager import ConfigManager
from curobo_ros.core.robot_model_manager import RobotModelManager
from curobo_ros.core.obstacle_manager import ObstacleManager
from curobo_ros.core.camera_system_manager import CameraSystemManager
from curobo_ros.core.ros_service_manager import RosServiceManager


def resolve_use_cuda_graph(node, default: bool = True) -> bool:
    """Resolve whether solvers should capture/replay CUDA graphs.

    Precedence: the ``CUROBO_USE_CUDA_GRAPH`` env var wins if set
    (``0``/``false``/``no``/``off`` -> False, anything else -> True), else the
    ROS ``use_cuda_graph`` parameter, else ``default``.

    Disabling graphs is the escape hatch for the MPC->Classic replay segfault
    (a stale captured MotionGen graph); the env var lets it be flipped without
    editing code or params.
    """
    env = os.environ.get('CUROBO_USE_CUDA_GRAPH')
    if env is not None:
        return env.strip().lower() not in ('0', 'false', 'no', 'off')
    if node is not None and node.has_parameter('use_cuda_graph'):
        return bool(node.get_parameter('use_cuda_graph').get_parameter_value().bool_value)
    return default


class ConfigWrapper(ABC):
    """
    Orchestrator class that wraps configuration and management for trajectory generation.

    This class uses composition to delegate responsibilities to specialized managers:
    - ConfigManager: Handles all configuration loading (robot, world, parameters)
    - RobotModelManager: Manages robot model and kinematics
    - ObstacleManager: Manages obstacles and voxel grid computation
    - CameraSystemManager: Manages camera configuration and setup
    - RosServiceManager: Manages ROS services, publishers, and timers

    This class serves as the base class for:
    - ConfigWrapperMotion
    - ConfigWrapperIK

    The reactive (MPC) solver is NOT a ConfigWrapper: it is built by
    MPCController directly from the shared ConfigWrapperMotion context, so all
    solvers share one robot/obstacles/scene/collision-cache.

    Child classes must implement:
    - update_world_config(node): Update world configuration based on obstacles
    - callback_get_collision_distance(node, request, response): Get collision distance
    """

    def __init__(self, node, robot):
        """
        Initialize ConfigWrapper with all managers.

        Args:
            node: ROS2 node instance
            robot: Robot interface object
        """
        self.node = node
        self.robot = robot

        # Phase 1: ConfigManager - Load all configurations
        self.config_manager = ConfigManager(node)

        # Phase 2: RobotModelManager - Manage robot model and kinematics
        self.robot_model_manager = RobotModelManager(
            self.config_manager.get_robot_config_file(),
            robot,
            self.config_manager.base_link,
            node=node,
        )

        # Adopt the canonical joint order/DOF from the built kinematics into the
        # descriptor, so every downstream consumer (strategies, context) is
        # DOF-agnostic and reads the real joint names instead of a hardcoded list.
        self.config_manager.get_robot_description().bind_kinematics(
            self.robot_model_manager.kin_model)

        # Phase 4: ObstacleManager - Manage obstacles (before RosServiceManager)
        # Pass initial Scene from ConfigManager to ObstacleManager
        self.obstacle_manager = ObstacleManager(
            node,
            self.config_manager,
            initial_scene=self.config_manager.get_scene(),
        )

        # Phase 5: CameraSystemManager - Manage cameras
        # Declare camera config parameter
        if not node.has_parameter('cameras_config_file'):
            node.declare_parameter('cameras_config_file', '')
        cameras_config_file = node.get_parameter('cameras_config_file').get_parameter_value().string_value
        self.camera_system_manager = CameraSystemManager(node, cameras_config_file)

        # Phase 3: RosServiceManager - Manage ROS services (last, depends on others)
        self.ros_service_manager = RosServiceManager(
            node,
            self.obstacle_manager,
            self.robot_model_manager,
            self.config_manager,
            self,   # Pass self so RosServiceManager can call update_world_config()
            robot   # Pass robot_context so RosServiceManager can expose get_robot_strategies
        )

        # Phase 6: Perception (Mapper) + propagation observers.
        # The Mapper turns camera data into an ESDF voxel grid used for
        # collision. The observers ensure every scene/cache mutation reaches
        # the solvers regardless of the caller (ROS service OR direct Python).
        num_cameras = 0
        camera_context = self.camera_system_manager.camera_context
        if camera_context is not None:
            num_cameras = len(camera_context.cameras)
        self.obstacle_manager.setup_perception(num_cameras=num_cameras)

        self._register_world_observers(node)

        # State information
        self.node_is_available = False
        if not node.has_parameter('node_is_available'):
            node.declare_parameter('node_is_available', False)

    def _register_world_observers(self, node):
        """Wire ObstacleManager mutations to solver propagation/rebuild.

        Prefers the node-level orchestration methods (UnifiedPlannerNode, which
        also refreshes IK), and falls back to this wrapper's own methods for the
        standalone planner node.
        """
        # World-change observer: propagate the updated Scene to all solvers.
        node_world_update = getattr(node, 'update_all_solvers_world', None)
        if node_world_update is not None:
            self.obstacle_manager.set_world_update_callback(node_world_update)
        else:
            self.obstacle_manager.set_world_update_callback(
                lambda: self.update_world_config(node)
            )

        # Cache-change observer: solvers must be rebuilt (the collision cache
        # size is fixed at solver creation, so a world update is not enough).
        node_cache_rebuild = getattr(node, 'rebuild_solvers_for_cache_change', None)
        if node_cache_rebuild is not None:
            self.obstacle_manager.set_collision_cache_callback(node_cache_rebuild)
        else:
            wrapper_rebuild = getattr(self, 'set_motion_gen_config', None)
            if wrapper_rebuild is not None:
                self.obstacle_manager.set_collision_cache_callback(
                    lambda: wrapper_rebuild(node, None, None)
                )

    def init_services(self, node=None):
        """Initialize ROS services - delegates to RosServiceManager

        Args:
            node: Optional node parameter for backward compatibility (not used)
        """
        self.ros_service_manager.init_services()

    # ==================== Properties for Backward Compatibility ====================
    # These properties allow child classes and external code to access manager
    # attributes as if they were direct attributes of ConfigWrapper

    @property
    def scene(self):
        """
        Get Scene from ObstacleManager (single source of truth).

        v2: Scene replaces v1's WorldConfig. Read-only access; mutations must
        go through ObstacleManager (add_object / remove_object / ...).
        """
        return self.obstacle_manager.get_scene()

    @property
    def robot_config_file(self):
        """Path to the robot YAML config — pass directly to v2 Cfg factories."""
        return self.config_manager.get_robot_config_file()

    @property
    def kin_model(self):
        """Get kinematics model from RobotModelManager"""
        return self.robot_model_manager.kin_model

    @property
    def base_link(self):
        """Get base link from ConfigManager"""
        return self.config_manager.base_link

    @property
    def obstacle_names(self):
        return self.obstacle_manager.obstacle_names

    # v2: single collision_cache integer (no per-type obb/mesh/blox split).
    @property
    def collision_cache(self):
        return self.obstacle_manager.collision_cache

    @collision_cache.setter
    def collision_cache(self, value):
        self.obstacle_manager.collision_cache = value

    @property
    def camera_context(self):
        """Get camera context from CameraSystemManager"""
        return self.camera_system_manager.camera_context

    @property
    def _ops_dtype(self):
        """Get operations dtype from RobotModelManager"""
        return self.robot_model_manager._ops_dtype

    @property
    def _device(self):
        """Get device from RobotModelManager"""
        return self.robot_model_manager._device

    @property
    def publish_collision_spheres_timer(self):
        """Get collision spheres timer from RosServiceManager"""
        return self.ros_service_manager.publish_collision_spheres_timer

    @publish_collision_spheres_timer.setter
    def publish_collision_spheres_timer(self, value):
        """Set collision spheres timer in RosServiceManager"""
        self.ros_service_manager.publish_collision_spheres_timer = value

    def publish_collision_spheres(self, node):
        """Delegate collision spheres publishing to RosServiceManager"""
        return self.ros_service_manager.publish_collision_spheres(node)


    # ==================== Abstract Methods ====================
    # These must be implemented by child classes (ConfigWrapperMotion, IK, MPC)

    @abstractmethod
    def update_world_config(self, node):
        """
        Update world configuration based on current obstacles.
        Must be implemented by child classes.

        Args:
            node: ROS2 node instance
        """
        raise NotImplementedError

    @abstractmethod
    def callback_get_collision_distance(self, node, request, response):
        """
        Get collision distance for current robot state.
        Must be implemented by child classes.

        Args:
            node: ROS2 node instance
            request: GetCollisionDistance request
            response: GetCollisionDistance response

        Returns:
            GetCollisionDistance response with collision distance data
        """
        raise NotImplementedError
