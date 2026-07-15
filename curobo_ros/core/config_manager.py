import os
import torch

from curobo.types import DeviceCfg
from curobo.scene import Scene
from curobo.config_io import load_yaml

from curobo_ros.robot.robot_description import load_robot_description


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
        self.robot_description = None

        # Load configurations in order (robot descriptor first: it supplies the
        # default base_link and the resolved robot config path).
        self._load_robot_description()
        self._load_ros_parameters()
        self._load_scene()
        self._load_robot_config()

    def _load_robot_description(self):
        """Load the selected robot descriptor (robots/<robot>.yaml or a path)."""
        if not self.node.has_parameter('robot'):
            self.node.declare_parameter('robot', 'doosan_m1013')
        robot = self.node.get_parameter('robot').get_parameter_value().string_value or 'doosan_m1013'
        self.robot_description = load_robot_description(robot)
        self.node.get_logger().info(
            f"ConfigManager loaded robot descriptor: {self.robot_description.display_name} "
            f"({self.robot_description.name})")

    def _load_ros_parameters(self):
        """Declare and load ROS parameters"""
        # base_link default comes from the descriptor (robot-specific), overridable.
        if not self.node.has_parameter('base_link'):
            self.node.declare_parameter('base_link', self.robot_description.base_link or 'base_0')
        if not self.node.has_parameter('world_file'):
            self.node.declare_parameter('world_file', '')

        self.base_link = self.node.get_parameter('base_link').get_parameter_value().string_value
        self.world_file = self.node.get_parameter('world_file').get_parameter_value().string_value

        self.node.get_logger().info(f'ConfigManager using base_link: {self.base_link}')

    def _load_scene(self):
        """Load Scene from world_file parameter or use an empty default."""
        if self.world_file:
            self.scene = Scene.create(load_yaml(self.world_file))
            self.node.get_logger().info(f'Loaded scene from: {self.world_file}')
        else:
            self.scene = Scene()
            self.node.get_logger().info('Using empty scene (obstacles will be added at runtime)')

    def _load_robot_config(self):
        """Resolve the robot YAML path and cache its dict form.

        Default = the descriptor's resolved cuRobo config (portable, paths already
        made absolute). ``robot_config_file`` stays as an explicit override.
        """
        # Default tensor args (kept for backward-compat consumers)
        self.tensor_args = DeviceCfg(device='cuda', dtype=torch.float32)

        default_robot_config = self.robot_description.curobo_config_path
        if not self.node.has_parameter('robot_config_file'):
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

    def get_robot_description(self):
        """Return the loaded RobotDescription (canonical robot metadata)."""
        return self.robot_description

    # ---- Getters ----

    def get_scene(self) -> Scene:
        """
        Return the initial Scene loaded from file/defaults.

        IMPORTANT: For the authoritative Scene with runtime obstacles, use
        obstacle_manager.get_scene() instead.
        """
        return self.scene

    def get_robot_config_file(self) -> str:
        """Return the resolved robot YAML path — v2 Cfg factories accept this directly."""
        return self.robot_config_file

    def get_robot_config_dict(self) -> dict:
        """Return the parsed robot config dict."""
        return self.robot_cfg_dict

    def get_base_link(self) -> str:
        return self.base_link
