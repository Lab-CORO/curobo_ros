import os
import torch
from ament_index_python.packages import get_package_share_directory

from curobo.types import DeviceCfg
from curobo.scene import Scene
from curobo.util_file import load_yaml


class ConfigManager:
    """
    Manages configuration loading for robot and world.
    Responsible for:
    - Loading ROS parameters
    - Loading INITIAL Scene from YAML (passed to ObstacleManager)
    - Loading robot configuration path/dict

    Note: the Scene is loaded here but ownership transfers to ObstacleManager.
          Use obstacle_manager.get_scene() to access the authoritative version.

    v2 notes:
    - WorldConfig → Scene (curobo.scene)
    - TensorDeviceType → DeviceCfg (curobo.types)
    - RobotConfig no longer built here: v2 factories accept a YAML path or
      dict via the `robot=` kwarg of MotionPlannerCfg.create(...).
    """

    def __init__(self, node):
        self.node = node

        # Scene parameters
        self.scene = None
        self.world_pose = [0, 0, 0, 1, 0, 0, 0]

        # ROS parameters
        self.base_link = None
        self.world_file = None
        self.robot_config_file = None
        self.robot_cfg_dict = None

        # Load configurations in order
        self._load_ros_parameters()
        self._load_scene()
        self._load_robot_config()

    def _load_ros_parameters(self):
        """Declare and load ROS parameters"""
        self.node.declare_parameter('base_link', 'base_0')
        self.node.declare_parameter('world_file', '')

        self.base_link = self.node.get_parameter('base_link').get_parameter_value().string_value
        self.world_file = self.node.get_parameter('world_file').get_parameter_value().string_value

        self.node.get_logger().info(f'ConfigManager using base_link: {self.base_link}')

    def _load_scene(self):
        """Load Scene from world_file parameter or use an empty default."""
        if self.world_file:
            self.scene = Scene.from_dict(load_yaml(self.world_file))
            self.node.get_logger().info(f'Loaded scene from: {self.world_file}')
        else:
            self.scene = Scene()
            self.node.get_logger().info('Using empty scene (obstacles will be added at runtime)')

    def _load_robot_config(self):
        """Resolve the robot YAML path and cache its dict form."""
        # Default tensor args (kept for backward-compat consumers)
        self.tensor_args = DeviceCfg(device='cuda', dtype=torch.float32)

        package_share_directory = get_package_share_directory('curobo_ros')
        default_robot_config = os.path.join(
            package_share_directory,
            'curobo_doosan',
            'src',
            'm1013',
            'm1013.yml',
        )
        self.node.declare_parameter('robot_config_file', default_robot_config)

        robot_config_file = self.node.get_parameter('robot_config_file').get_parameter_value().string_value
        if not robot_config_file:
            robot_config_file = default_robot_config

        self.robot_config_file = robot_config_file

        config_file = load_yaml(robot_config_file)
        robot_cfg_dict = config_file.get('robot_cfg', config_file)
        robot_cfg_dict.pop('cspace', None)
        self.robot_cfg_dict = robot_cfg_dict

        self.node.get_logger().info(f'Loaded robot config from: {robot_config_file}')

    # ---- Getters ----

    def get_scene(self) -> Scene:
        """
        Return the initial Scene loaded from file/defaults.

        IMPORTANT: For the authoritative Scene with runtime obstacles, use
        obstacle_manager.get_scene() instead.
        """
        return self.scene

    # Legacy alias for code that still calls get_world_config()
    def get_world_config(self) -> Scene:
        return self.scene

    def get_robot_config_file(self) -> str:
        """Return the resolved robot YAML path — v2 Cfg factories accept this directly."""
        return self.robot_config_file

    def get_robot_config_dict(self) -> dict:
        """Return the parsed robot config dict."""
        return self.robot_cfg_dict

    def get_base_link(self) -> str:
        return self.base_link
