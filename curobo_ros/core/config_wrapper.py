import os

from curobo_msgs.srv import AddObject

from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.types import WorldConfig
from curobo.util_file import load_yaml, join_path, get_world_configs_path
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig

from ament_index_python.packages import get_package_share_directory


class ConfigWrapper:
    '''
    This class is used to wrap the configuration of the robot and the world for the trajectory generation class.
    It removes responsibilities relative to motion generation configuration from the main node.
    The original class uses the "Visitor" pattern to access these functionalities without splitting ownership of the node.
    '''

    def __init__(self):
        # Motion generation parameters
        self.trajopt_tsteps = 32
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

        # World config parameters
        self.world_cfg = None
        self.world_pose = [0, 0, 0, 1, 0, 0, 0]
        self.world_integrator_type = "occupancy"

        # Load robot configuration and other configuration files
        config_file_path = os.path.join(get_package_share_directory(
            "curobo_ros"), 'curobo_doosan/src/m1013/m1013.yml')
        self.robot_cfg = load_yaml(config_file_path)["robot_cfg"]

        self.j_names = self.robot_cfg["kinematics"]["cspace"]["joint_names"]
        self.default_config = self.robot_cfg["kinematics"]["cspace"]["retract_config"]

        self.world_cfg_table = WorldConfig.from_dict(
            load_yaml(join_path(get_world_configs_path(), "collision_wall.yml")))

    def set_motion_gen_config(self, node, request, response):
        self.world_cfg = WorldConfig.from_dict(
            {
                "blox": {
                    "world": {
                        "pose": self.world_pose,
                        "integrator_type": self.world_integrator_type,
                        "voxel_size":  node.get_parameter('voxel_size').get_parameter_value().double_value,
                    },
                },
            }
        )

        # TODO: Use the service to add objects instead
        self.world_cfg_table.cuboid[0].pose[2] -= 0.01
        self.world_cfg.add_obstacle(self.world_cfg_table.cuboid[0])
        self.world_cfg.add_obstacle(self.world_cfg_table.cuboid[1])

        motion_gen_config = MotionGenConfig.load_from_robot_config(
            self.robot_cfg,
            self.world_cfg,
            node.tensor_args,
            trajopt_tsteps=self.trajopt_tsteps,
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

        node.motion_gen = MotionGen(motion_gen_config)

        node.get_logger().info("warming up..")

        node.motion_gen.warmup()

        node.world_model = node.motion_gen.world_collision

        node.get_logger().info("Motion generation config set")

        if response is not None:
            response.success = True
            response.message = "Motion generation config set"

        return response

    def callback_add_object(self, node, request, response):
        print(f"Adding object from {node.get_name()}")
        response.success = True
        return response
