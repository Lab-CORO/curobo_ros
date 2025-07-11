import os
from abc import ABC, abstractmethod

from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
from curobo.types.base import TensorDeviceType
from curobo.types.robot import RobotConfig
from curobo.types.state import JointState
from geometry_msgs.msg import Vector3

from curobo.geom.types import WorldConfig, Capsule, Cylinder, Sphere, Mesh
from curobo.util_file import load_yaml

from ament_index_python.packages import get_package_share_directory
import ros2_numpy as rnp
import numpy as np

from functools import partial
import torch



from std_srvs.srv import Trigger
from curobo_msgs.srv import AddObject, RemoveObject, GetVoxelGrid, GetCollisionDistance
from curobo.geom.types import Cuboid
from visualization_msgs.msg import MarkerArray, Marker


class ConfigWrapper:
    '''
    This class is used to wrap the configuration of the robot and the world for the trajectory generation class.
    It removes responsibilities relative to motion generation configuration from the main node.
    The original class uses the "Visitor" pattern to access these functionalities without splitting ownership of the node.
    '''

    def __init__(self, node, robot):

       
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


        # Initialize tensor arguments with CUDA device
        tensor_args = TensorDeviceType(device='cuda', dtype=torch.float32)

        # Get the path to your curobo_ros package
        package_share_directory = get_package_share_directory('curobo_ros')
        node.declare_parameter('robot_config_file',  os.path.join(package_share_directory, 'curobo_doosan', 'src', 'm1013', 'm1013.yml'))
        robot_config_file = node.get_parameter('robot_config_file').get_parameter_value().string_value
        config_file = load_yaml(robot_config_file)
        robot_cfg_dict = config_file["robot_cfg"]
        robot_cfg_dict.pop('cspace', None)
        self.robot_cfg = RobotConfig.from_dict(robot_cfg_dict, tensor_args)
        urdf_file = self.robot_cfg.kinematics.generator_config.urdf_path

        if not os.path.isabs(urdf_file):
            urdf_file = os.path.join(package_share_directory, 'curobo_doosan', 'src', 'm1013', urdf_file)
            self.robot_cfg.kinematics.generator_config.urdf_path = urdf_file
            print(self.robot_cfg)
        self.kin_model = CudaRobotModel(self.robot_cfg.kinematics)

        # Interface with robot information
        self.robot = robot
        # for distance collision checker
        self._ops_dtype = torch.float32
        self._device = torch.device('cuda')

        # State information
        self.node_is_available = False
        node.declare_parameter('node_is_available', False)

    def init_services(self, node):
         # Add all services

        self.add_object_srv = node.create_service(
            AddObject, node.get_name() + '/add_object', partial(self.callback_add_object, node))

        self.add_object_srv = node.create_service(
            RemoveObject, node.get_name() + '/remove_object', partial(self.callback_remove_object, node))

        self.add_object_srv = node.create_service(
            Trigger, node.get_name() + '/get_obstacles', partial(self.callback_get_obstacles, node))

        self.node_available_srv = node.create_service(
            Trigger, node.get_name() + '/is_available', partial(self.callback_is_available, node))

        self.remove_all_objects_srv = node.create_service(
            Trigger, node.get_name() + '/remove_all_objects', partial(self.callback_remove_all_objects, node))

        self.get_voxel_map_srv = node.create_service(
            GetVoxelGrid, node.get_name() + '/get_voxel_grid', partial(self.callback_get_voxel_grid, node))

        self.get_collision_distance_srv = node.create_service(
            GetCollisionDistance, node.get_name() + '/get_collision_distance', partial(self.callback_get_collision_distance, node))

        # Add publisher
        self.publish_collision_spheres_pub = node.create_publisher(MarkerArray, node.get_name() + '/collision_spheres', 1)

        # Add timer 
        self.publish_collision_spheres_timer = node.create_timer(0.5, partial(self.publish_collision_spheres, node))

    def callback_add_object(self, node, request: AddObject, response):
        '''
        This function is called by the service callback created in the node.
        It adds an object to the world configuration.
        The object is created based on the type, pose, dimensions and color requested.
        The object is then added to the world configuration.
        Note: Since cuRobo currently only supports Cuboid object manipulation,
        all objects are converted to a Cuboid object before being added.
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


        # Extract the values from the request
        extracted_pose = [request.pose.position.x, request.pose.position.y, request.pose.position.z,
                          request.pose.orientation.w, request.pose.orientation.x, request.pose.orientation.y, request.pose.orientation.z]

        # Extracted dimensions are interpreted differently based on the object type
        # CUBOID: [x, y, z]
        # CAPSULE: [radius, _, _]
        # CYLINDER: [radius, height, _]
        # SPHERE: [radius, _, _]
        # MESH: [scale_x, scale_y, scale_z]
        extracted_dimensions = [request.dimensions.x,
                                request.dimensions.y, request.dimensions.z]

        extracted_color = [request.color.r, request.color.g,
                           request.color.b, request.color.a]

        # node.get_logger().info(
        #     f"Adding object {request.name} for {node.get_name()}")

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

            case request.MESH:
                obstacle = Mesh(
                    name=request.name,
                    pose=extracted_pose,
                    file_path=request.mesh_file_path,
                    scale=extracted_dimensions
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

    def callback_get_obstacles(self, node, request: Trigger, response):
        for world_object in self.world_cfg.objects:
            response.message = response.message + world_object.name
        return response

    def callback_is_available(self, node, request: Trigger, response):
        response.success = self.node_is_available
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
            self.world_cfg.remove_obstacle(request.name)
            self.world_cfg.cuboid = list(
                filter(lambda obj: obj.name != request.name, self.world_cfg.cuboid))
            self.update_world_config(node)

        except Exception as e:
            response.success = False
            response.message = 'Object ' + request.name + \
                ' failed to be removed: ' + str(e)
            return response

        response.success = True
        response.message = 'Object ' + request.name + ' removed successfully'

        return response

    def callback_remove_all_objects(self, node, _, response):
        '''
        This function is called by the service callback created in the node.
        It removes all objects from the world configuration.
        Since cuRobo only supports Cuboid object manipulation, it only needs to remove objects of that type.
        '''
        # Get objects to remove
        # Targeting only Cuboids protects any Blox that may have been added by cameras
        for world_object in self.world_cfg.cuboid:
            self.world_cfg.remove_obstacle(world_object.name)

        # Empty the list of cuboids that lingers in the world config
        self.world_cfg.cuboid = []

        self.update_world_config(node)

        response.success = True
        response.message = 'All objects removed successfully'
        return response

    def callback_get_voxel_grid(self, node, request: GetVoxelGrid, response):
        """
        Get the worldConfig VoxelGrid and send it as message.

        Args:
            node (Node): Node object.
            request (GetVoxelGrid): Request object.
            response (GetVoxelGridResponse): Response object.

        Returns:
            GetVoxelGridResponse: Response object.
        """
        # response.success = True

        voxel_size = self.world_cfg.blox[0].voxel_size
        min_x, min_y, min_z = float('inf'), float('inf'), float('inf')
        max_x, max_y, max_z = float('-inf'), float('-inf'), float('-inf')

        for cuboid in self.world_cfg.cuboid:
            cuboid_min = np.array(cuboid.pose[:3]) - np.array(cuboid.dims) / 2
            cuboid_max = np.array(cuboid.pose[:3]) + np.array(cuboid.dims) / 2

            min_x, min_y, min_z = np.minimum([min_x, min_y, min_z], cuboid_min)
            max_x, max_y, max_z = np.maximum([max_x, max_y, max_z], cuboid_max)

        max_x = 1.5
        max_y = 1.5
        max_z = 1.5
        min_x = -1.5
        min_y = -1.5
        min_z = -1.5
        grid_size_x = int(np.ceil((max_x - min_x) / voxel_size))
        grid_size_y = int(np.ceil((max_y - min_y) / voxel_size))
        grid_size_z = int(np.ceil((max_z - min_z) / voxel_size))

        voxel_grid = np.zeros((grid_size_x, grid_size_y, grid_size_z), dtype=np.uint8)

        for cuboid in self.world_cfg.cuboid:
            cuboid_min = np.array(cuboid.pose[:3]) - np.array(cuboid.dims) / 2
            cuboid_max = np.array(cuboid.pose[:3]) + np.array(cuboid.dims) / 2

            min_voxel_idx = np.floor((cuboid_min - [min_x, min_y, min_z]) / voxel_size).astype(int)
            max_voxel_idx = np.ceil((cuboid_max - [min_x, min_y, min_z]) / voxel_size).astype(int)

            voxel_grid[
                min_voxel_idx[0]:max_voxel_idx[0],
                min_voxel_idx[1]:max_voxel_idx[1],
                min_voxel_idx[2]:max_voxel_idx[2]
            ] = 1  # Mark as occupied

        response.voxel_grid.resolutions = rnp.msgify(Vector3, np.array([voxel_size, voxel_size, voxel_size]))
        response.voxel_grid.size_x = grid_size_x
        response.voxel_grid.size_y = grid_size_y
        response.voxel_grid.size_z = grid_size_z

        # Set the origin
        response.voxel_grid.origin.x = min_x
        response.voxel_grid.origin.y = min_y
        response.voxel_grid.origin.z = min_z
  
        # Flatten the voxel grid into a 1D list and assign it to the message's data field
        response.voxel_grid.data = voxel_grid.flatten().tolist()

        return response

    
    def callback_get_collision_distance(self, node, request: GetCollisionDistance, response):
        raise NotImplementedError

    @abstractmethod
    def update_world_config(self, node):
        raise NotImplementedError


    def publish_collision_spheres(self, node):
        """
        Publishes the robot's collision spheres as markers for visualization in RViz.
        Useful for debugging and ensuring proper masking of the robot in the point cloud.
        """
        q_js =JointState(position=torch.tensor(self.robot.get_joint_pose(), dtype=self._ops_dtype, device=self._device),
                        joint_names=['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'])
        kinematics_state = self.kin_model.get_state(q_js.position)
        robot_spheres = kinematics_state.link_spheres_tensor.view(-1, 4)
        robot_spheres = robot_spheres.cpu().numpy().tolist()
        marker_array = MarkerArray()
        for i, sphere in enumerate(robot_spheres):
            marker = Marker()
            marker.header.frame_id = 'base_0'
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.id = i
            marker.pose.position.x = sphere[0]
            marker.pose.position.y = sphere[1]
            marker.pose.position.z = sphere[2]
            marker.scale.x = sphere[3] * 2  # Diameter
            marker.scale.y = sphere[3] * 2
            marker.scale.z = sphere[3] * 2
            marker.color.a = 0.5  # Transparency
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)
        self.publish_collision_spheres_pub.publish(marker_array)
