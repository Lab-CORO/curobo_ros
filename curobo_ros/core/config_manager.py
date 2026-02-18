import os
from curobo.types.base import TensorDeviceType
from curobo.types.robot import RobotConfig
from curobo.geom.types import WorldConfig
from curobo.util_file import load_yaml
from ament_index_python.packages import get_package_share_directory
import torch


class ConfigManager:
    """
    Manages configuration loading for robot and world.
    Responsible for:
    - Loading ROS parameters
    - Loading INITIAL world configuration from YAML (passed to ObstacleManager)
    - Loading robot configuration from YAML

    Note: world_cfg is loaded here but ownership transfers to ObstacleManager.
          Use obstacle_manager.get_world_cfg() to access the authoritative version.
    """

    def __init__(self, node):
        self.node = node

        # World config parameters
        self.world_cfg = None
        self.world_pose = [0, 0, 0, 1, 0, 0, 0]
        self.world_integrator_type = "occupancy"

        # ROS parameters
        self.base_link = None
        self.world_file = None
        self.robot_cfg = None

        # Load configurations in order
        self._load_ros_parameters()
        self._load_world_config()
        self._load_robot_config()

    def _load_ros_parameters(self):
        """Declare and load ROS parameters"""
        # Get base_link from ROS parameter, default to 'base_0'
        self.node.declare_parameter('base_link', 'base_0')
        self.node.declare_parameter('world_file', '')

        self.base_link = self.node.get_parameter('base_link').get_parameter_value().string_value
        self.world_file = self.node.get_parameter('world_file').get_parameter_value().string_value

        self.node.get_logger().info(f'ConfigManager using base_link: {self.base_link}')

    def _load_world_config(self):
        """Load WorldConfig from world_file parameter or use default configuration"""
        if self.world_file:
            self.world_cfg = WorldConfig.from_dict(load_yaml(self.world_file))
            self.node.get_logger().info(f'Loaded world config from: {self.world_file}')
        else:
            self.world_cfg = WorldConfig.from_dict(
                {
                    "blox": {
                        "world": {
                            "pose": self.world_pose,
                            "integrator_type": self.world_integrator_type
                        },
                    },
                }
            )
            self.node.get_logger().info('Using default world configuration')

    def _load_robot_config(self):
        """Load RobotConfig from robot_config_file parameter"""
        # Initialize tensor arguments with CUDA device
        tensor_args = TensorDeviceType(device='cuda', dtype=torch.float32)

        # Get the path to curobo_ros package
        package_share_directory = get_package_share_directory('curobo_ros')

        # Declare parameter with default path
        default_robot_config = os.path.join(
            package_share_directory,
            'curobo_doosan',
            'src',
            'm1013',
            'm1013.yml'
        )
        self.node.declare_parameter('robot_config_file', default_robot_config)

        robot_config_file = self.node.get_parameter('robot_config_file').get_parameter_value().string_value

        if not robot_config_file:
            robot_config_file = default_robot_config

        # Load and parse robot configuration
        config_file = load_yaml(robot_config_file)
        robot_cfg_dict = config_file["robot_cfg"]
        robot_cfg_dict.pop('cspace', None)

        self.robot_cfg = RobotConfig.from_dict(robot_cfg_dict, tensor_args)

        self.node.get_logger().info(f'Loaded robot config from: {robot_config_file}')

    def get_world_config(self) -> WorldConfig:
        """
        Get the INITIAL world configuration loaded from file/defaults.

        IMPORTANT: This returns the initial configuration only.
        For the authoritative world_cfg with obstacles, use:
            obstacle_manager.get_world_cfg()

        Returns:
            WorldConfig: Initial world configuration (without runtime obstacles)
        """
        return self.world_cfg

    def get_robot_config(self) -> RobotConfig:
        """Get the robot configuration"""
        return self.robot_cfg

    def get_base_link(self) -> str:
        """Get the base link frame name"""
        return self.base_link
