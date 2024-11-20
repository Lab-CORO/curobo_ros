import os

from curobo_msgs.srv import AddObject, RemoveObject

from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.types import WorldConfig, Cuboid, Capsule, Cylinder, Sphere
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
        # TODO Have these values be able to be overwritten in a launch file
        # Motion generation parameters
        self.trajopt_tsteps = 32
        self.collision_cache = {"obb": 10}
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

        # Set the world configuration
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

        # Load robot configuration and other configuration files
        config_file_path = os.path.join(get_package_share_directory(
            "curobo_ros"), 'curobo_doosan/src/m1013/m1013.yml')
        self.robot_cfg = load_yaml(config_file_path)["robot_cfg"]

        self.j_names = self.robot_cfg["kinematics"]["cspace"]["joint_names"]
        self.default_config = self.robot_cfg["kinematics"]["cspace"]["retract_config"]

        self.world_cfg_table = WorldConfig.from_dict(
            load_yaml(join_path(get_world_configs_path(), "collision_wall.yml")))

    def set_motion_gen_config(self, node, request, response):
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

        # Set the motion generation configuration in the node and warmup the motion generation
        node.motion_gen = MotionGen(motion_gen_config)

        node.get_logger().info("warming up..")

        node.motion_gen.warmup()

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

    def callback_add_object(self, node, request: AddObject, response):
        '''
        This function is called by the service callback created in the node.
        It adds an object to the world configuration.
        The object is created based on the type, pose, dimensions and color requested.
        The object is then added to the world configuration.
        '''
        # Check for object name uniqueness
        for world_object in self.world_cfg.objects:
            if world_object.name == request.name:
                response.success = False
                response.message = 'Object with name "' + request.name + '" already exists'
                return response

        # Check for object dimensions validity
        if request.dimensions.x <= 0 or request.dimensions.y <= 0 or request.dimensions.z <= 0:
            response.success = False
            response.message = 'Object dimensions must be positive'
            return response

        # Check for object position validity
        if request.pose.position.z - request.dimensions.z / 2 < 0:
            response.success = False
            response.message = 'Object must be above the ground'
            return response

        # Extract the values from the request
        extracted_pose = [request.pose.position.x, request.pose.position.y, request.pose.position.z,
                          request.pose.orientation.w, request.pose.orientation.x, request.pose.orientation.y, request.pose.orientation.z]

        # Extracted dimensions are interpreted differently based on the object type
        # CUBOID: [x, y, z]
        # CAPSULE: [radius, _, _]
        # CYLINDER: [radius, height, _]
        # SPHERE: [radius, _, _]
        extracted_dimensions = [request.dimensions.x,
                                request.dimensions.y, request.dimensions.z]

        extracted_color = [request.color.r, request.color.g,
                           request.color.b, request.color.a]

        node.get_logger().info(
            f"Adding object {request.name} for {node.get_name()}")

        response.success = True
        obstacle = None

        # Create the object based on the type requested
        match request.type:
            case request.CUBOID:
                obstacle = Cuboid(
                    name=request.name,
                    pose=extracted_pose,
                    dims=extracted_dimensions,
                    color=extracted_color,
                )

            case request.CAPSULE:
                obstacle = Capsule(
                    name=request.name,
                    pose=extracted_pose,
                    base=[0, 0, 0],
                    tip=[0, 0, extracted_dimensions[1]],
                    radius=extracted_dimensions[0],
                    color=extracted_color,
                ).get_cuboid()

            case request.CYLINDER:
                obstacle = Cylinder(
                    name=request.name,
                    pose=extracted_pose,
                    radius=extracted_dimensions[0],
                    height=extracted_dimensions[1],
                    color=extracted_color,
                ).get_cuboid()

            case request.SPHERE:
                obstacle = Sphere(
                    name=request.name,
                    pose=extracted_pose,
                    radius=extracted_dimensions[0],
                    color=extracted_color,
                ).get_cuboid()

            case _:  # default
                response.success = False
                response.message = 'Object type "' + \
                    str(request.type) + '" not recognized'

        if response.success:
            self.world_cfg.add_obstacle(obstacle)
            self.update_world_config(node)
            response.message = 'Object ' + request.name + ' added successfully'

        return response

    def callback_remove_object(self, node, request: RemoveObject, response):
        '''
        This function is called by the service callback created in the node.
        It removes an object from the world configuration.
        The object is removed based on the name requested.
        '''
        object_exists = False

        # Check that the object exists
        for world_object in self.world_cfg.objects:
            if world_object.name == request.name:
                object_exists = True
                break

        if not object_exists:
            response.success = False
            response.message = 'Object ' + request.name + ' does not exist'
            return response

        try:
            # TODO Figure out why this is failing to remove it in the WorldCollision object
            # but is properly removed from the WorldConfig obstacle list
            self.world_cfg.remove_obstacle(request.name)

        except Exception as e:
            response.success = False
            response.message = 'Object ' + request.name + \
                ' failed to be removed: ' + str(e)
            return response

        self.update_world_config(node)
        response.success = True
        response.message = 'Object ' + request.name + ' removed successfully'
        return response
