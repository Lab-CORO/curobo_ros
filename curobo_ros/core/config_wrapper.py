from abc import ABC, abstractmethod

# Import all manager classes
from curobo_ros.core.config_manager import ConfigManager
from curobo_ros.core.robot_model_manager import RobotModelManager
from curobo_ros.core.obstacle_manager import ObstacleManager
from curobo_ros.core.camera_system_manager import CameraSystemManager
from curobo_ros.core.ros_service_manager import RosServiceManager


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
    - ConfigWrapperMPC

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
            self.config_manager.robot_cfg,
            robot,
            self.config_manager.base_link
        )

        # Phase 4: ObstacleManager - Manage obstacles (before RosServiceManager)
        self.obstacle_manager = ObstacleManager(node, self.config_manager)

        # Phase 5: CameraSystemManager - Manage cameras
        # Declare camera config parameter
        node.declare_parameter('cameras_config_file', '')
        cameras_config_file = node.get_parameter('cameras_config_file').get_parameter_value().string_value
        self.camera_system_manager = CameraSystemManager(node, cameras_config_file)

        # Phase 3: RosServiceManager - Manage ROS services (last, depends on others)
        self.ros_service_manager = RosServiceManager(
            node,
            self.obstacle_manager,
            self.robot_model_manager,
            self.config_manager,
            self  # Pass self so RosServiceManager can call update_world_config()
        )

        # State information
        self.node_is_available = False
        node.declare_parameter('node_is_available', False)

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
    def world_cfg(self):
        """Get world configuration from ConfigManager"""
        return self.config_manager.world_cfg

    @world_cfg.setter
    def world_cfg(self, value):
        """Set world configuration in ConfigManager"""
        self.config_manager.world_cfg = value

    @property
    def robot_cfg(self):
        """Get robot configuration from ConfigManager"""
        return self.config_manager.robot_cfg

    @property
    def kin_model(self):
        """Get kinematics model from RobotModelManager"""
        return self.robot_model_manager.kin_model

    @property
    def base_link(self):
        """Get base link from ConfigManager"""
        return self.config_manager.base_link

    @property
    def cuboid_list(self):
        """Get cuboid obstacle list from ObstacleManager"""
        return self.obstacle_manager.cuboid_list

    @property
    def mesh_list(self):
        """Get mesh obstacle list from ObstacleManager"""
        return self.obstacle_manager.mesh_list

    @property
    def obstacle_names(self):
        """Get obstacle names from ObstacleManager"""
        return self.obstacle_manager.obstacle_names

    @property
    def current_collision_checker(self):
        """Get current collision checker type from ObstacleManager"""
        return self.obstacle_manager.current_collision_checker

    @current_collision_checker.setter
    def current_collision_checker(self, value):
        """Set current collision checker type in ObstacleManager"""
        self.obstacle_manager.current_collision_checker = value

    @property
    def collision_checker_type(self):
        """Get collision checker type from ObstacleManager"""
        return self.obstacle_manager.collision_checker_type

    @collision_checker_type.setter
    def collision_checker_type(self, value):
        """Set collision checker type in ObstacleManager (updates cache automatically)"""
        self.obstacle_manager.set_collision_checker_type(value)

    @property
    def collision_cache(self):
        """Get collision cache from ObstacleManager"""
        return self.obstacle_manager.collision_cache

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

    def _update_collision_cache(self):
        """Update collision cache - delegates to ObstacleManager"""
        self.obstacle_manager._update_collision_cache()

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
