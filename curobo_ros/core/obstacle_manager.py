import os
import numpy as np
import torch
import ros2_numpy as rnp
from std_srvs.srv import Trigger
from curobo_msgs.srv import AddObject, RemoveObject, GetVoxelGrid
from curobo.geom.types import Cuboid, Capsule, Cylinder, Sphere, Mesh
from curobo.geom.sdf.world import CollisionCheckerType
from geometry_msgs.msg import Vector3


class ObstacleManager:
    """
    Manages obstacles and voxel grid computation.
    Responsible for:
    - Managing obstacle lists (cuboid_list, mesh_list, obstacle_names)
    - Adding/removing obstacles with validation
    - Converting primitive geometries to cuboids
    - Computing voxel grids for collision checking
    """

    def __init__(self, node, config_manager):
        """
        Initialize obstacle manager.

        Args:
            node: ROS2 node instance
            config_manager: ConfigManager for accessing world_cfg
        """
        self.node = node
        self.config_manager = config_manager

        # Separate lists for different obstacle types
        # This allows us to preserve mesh geometry when using MESH collision checker
        # and convert to cuboids when using BLOX collision checker
        self.cuboid_list = []  # Stores cuboid and primitive obstacles
        self.mesh_list = []    # Stores mesh obstacles with full geometry
        self.obstacle_names = []  # Track all obstacle names for uniqueness check

        # Collision checker configuration
        # Default collision checker (can be changed dynamically)
        self.collision_checker_type = CollisionCheckerType.BLOX
        self.current_collision_checker = self.collision_checker_type

        # Adaptive collision cache based on checker type
        self.collision_cache = None
        self._update_collision_cache()

    def add_object(self, node, request: AddObject, response):
        """
        Add an object to the world configuration.
        The object is created based on the type, pose, dimensions and color requested.

        IMPORTANT: Mesh obstacles are stored separately in mesh_list to preserve geometry.
        When MESH collision checker is active, mesh geometry is used directly.
        When BLOX collision checker is active, meshes are converted to cuboid bounding boxes.
        """
        # Check for object name uniqueness
        if request.name in self.obstacle_names:
            response.success = False
            response.message = 'Object with name "' + request.name + '" already exists'
            return response

        # Check for object dimensions validity
        if request.dimensions.x <= 0 or request.dimensions.y <= 0 or request.dimensions.z <= 0:
            response.success = False
            response.message = 'Object dimensions must be positive'
            return response

        # Extract the values from the request
        extracted_pose = [
            request.pose.position.x, request.pose.position.y, request.pose.position.z,
            request.pose.orientation.w, request.pose.orientation.x,
            request.pose.orientation.y, request.pose.orientation.z
        ]

        # Extracted dimensions are interpreted differently based on the object type
        # CUBOID: [x, y, z]
        # CAPSULE: [radius, _, _]
        # CYLINDER: [radius, height, _]
        # SPHERE: [radius, _, _]
        # MESH: [scale_x, scale_y, scale_z]
        extracted_dimensions = [
            request.dimensions.x,
            request.dimensions.y,
            request.dimensions.z
        ]

        extracted_color = [
            request.color.r, request.color.g,
            request.color.b, request.color.a
        ]

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
                self.cuboid_list.append(obstacle)
                self.obstacle_names.append(request.name)

            case request.CAPSULE:
                obstacle = Capsule(
                    name=request.name,
                    pose=extracted_pose,
                    base=[0, 0, 0],
                    tip=[0, 0, extracted_dimensions[1]],
                    radius=extracted_dimensions[0],
                    color=extracted_color,
                ).get_cuboid()
                self.cuboid_list.append(obstacle)
                self.obstacle_names.append(request.name)

            case request.CYLINDER:
                obstacle = Cylinder(
                    name=request.name,
                    pose=extracted_pose,
                    radius=extracted_dimensions[0],
                    height=extracted_dimensions[1],
                    color=extracted_color,
                ).get_cuboid()
                self.cuboid_list.append(obstacle)
                self.obstacle_names.append(request.name)

            case request.SPHERE:
                obstacle = Sphere(
                    name=request.name,
                    pose=extracted_pose,
                    radius=extracted_dimensions[0],
                    color=extracted_color,
                ).get_cuboid()
                self.cuboid_list.append(obstacle)
                self.obstacle_names.append(request.name)

            case request.MESH:
                # Check file exists
                if not os.path.exists(request.mesh_file_path):
                    response.success = False
                    response.message = f"Mesh file not found: {request.mesh_file_path}"
                    return response

                # Create Mesh object WITHOUT calling .get_cuboid()
                # This preserves the mesh geometry for MESH collision checker
                obstacle = Mesh(
                    name=request.name,
                    pose=extracted_pose,
                    file_path=request.mesh_file_path,
                    scale=extracted_dimensions
                )
                self.mesh_list.append(obstacle)
                self.obstacle_names.append(request.name)

                node.get_logger().info(
                    f"Added MESH obstacle '{request.name}' "
                    f"(current checker: {self.current_collision_checker})"
                )

            case _:  # default
                response.success = False
                response.message = 'Object type "' + str(request.type) + '" not recognized'

        if response.success:
            response.message = 'Object ' + request.name + ' added successfully'

        return response

    def remove_object(self, node, request: RemoveObject, response):
        """
        Remove an object from the world configuration.
        The object is removed based on the name requested.
        Searches both cuboid_list and mesh_list.
        """
        found = False

        # Search in cuboid list
        for i, obs in enumerate(self.cuboid_list):
            if obs.name == request.name:
                self.cuboid_list.pop(i)
                found = True
                break

        # Search in mesh list if not found in cuboid list
        if not found:
            for i, obs in enumerate(self.mesh_list):
                if obs.name == request.name:
                    self.mesh_list.pop(i)
                    found = True
                    break

        if not found:
            response.success = False
            response.message = f"Object '{request.name}' not found"
            return response

        try:
            # Remove from obstacle names list
            self.obstacle_names.remove(request.name)

            response.success = True
            response.message = f"Object '{request.name}' removed successfully"

        except Exception as e:
            response.success = False
            response.message = f"Object '{request.name}' failed to be removed: {str(e)}"
            return response

        return response

    def remove_all_objects(self, node, request: Trigger, response):
        """
        Remove all objects from the world configuration.
        Since cuRobo only supports Cuboid object manipulation, it only needs to remove objects of that type.
        """
        world_cfg = self.config_manager.world_cfg

        # Get objects to remove
        # Targeting only Cuboids protects any Blox that may have been added by cameras
        for world_object in world_cfg.cuboid:
            world_cfg.remove_obstacle(world_object.name)

        # Empty the list of cuboids that lingers in the world config
        world_cfg.cuboid = []

        response.success = True
        response.message = 'All objects removed successfully'
        return response

    def get_obstacles(self, node, request: Trigger, response):
        """Get list of all obstacle names"""
        world_cfg = self.config_manager.world_cfg
        for world_object in world_cfg.objects:
            response.message = response.message + world_object.name
        return response

    def get_voxel_grid(self, node, request: GetVoxelGrid, response):
        """
        Get the WorldConfig VoxelGrid and send it as message.

        Args:
            node: Node object.
            request: GetVoxelGrid request.
            response: GetVoxelGridResponse object.

        Returns:
            GetVoxelGridResponse: Response object with voxel grid data.
        """
        voxel_size = node.get_parameter('voxel_size').get_parameter_value().double_value
        min_x, min_y, min_z = float('inf'), float('inf'), float('inf')
        max_x, max_y, max_z = float('-inf'), float('-inf'), float('-inf')

        max_x = 1.52
        max_y = 1.52
        max_z = 1.52
        min_x = -1.52
        min_y = -1.52
        min_z = -1.52

        grid_size_x = int(np.ceil((max_x - min_x) / voxel_size))
        grid_size_y = int(np.ceil((max_y - min_y) / voxel_size))
        grid_size_z = int(np.ceil((max_z - min_z) / voxel_size))

        voxel_grid = np.zeros((grid_size_x, grid_size_y, grid_size_z), dtype=np.uint32)

        bounding = Cuboid("t", dims=[10, 10, 10.0], pose=[0, 0, 0, 1, 0, 0, 0])
        voxels = self.node.world_model.get_voxels_in_bounding_box(bounding, voxel_size)

        # Add voxels from world model to the voxel grid
        if voxels is not None and len(voxels) > 0:
            # Convert voxels tensor to numpy
            if torch.is_tensor(voxels):
                voxels_np = voxels.cpu().numpy()
            else:
                voxels_np = np.array(voxels)

            # Each voxel should have xyz coordinates
            # Convert world coordinates to grid indices
            for voxel in voxels_np:
                if len(voxel) >= 3:
                    voxel_x, voxel_y, voxel_z = voxel[:3]

                    # Convert to grid indices
                    grid_x = int(np.floor((voxel_x - min_x) / voxel_size))
                    grid_y = int(np.floor((voxel_y - min_y) / voxel_size))
                    grid_z = int(np.floor((voxel_z - min_z) / voxel_size))

                    # Check bounds and mark as occupied
                    if (0 <= grid_x < grid_size_x and
                        0 <= grid_y < grid_size_y and
                        0 <= grid_z < grid_size_z):
                        voxel_grid[grid_x, grid_y, grid_z] = 1

            self.node.get_logger().info(f"Added {len(voxels_np)} voxels from world model to voxel grid")

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

    def _update_collision_cache(self):
        """
        Update collision cache based on checker type.

        Different collision checkers require different cache configurations:
        - MESH checker: Uses OBB (Oriented Bounding Box) + mesh cache
        - BLOX/PRIMITIVE checkers: Use OBB + blox voxel cache
        """
        if self.collision_checker_type == CollisionCheckerType.MESH:
            self.collision_cache = {'obb': 100, 'mesh': 10}
        else:  # BLOX or PRIMITIVE
            self.collision_cache = {'obb': 100, 'blox': 10}

    def set_collision_checker_type(self, checker_type):
        """
        Set the collision checker type and update cache accordingly.

        Args:
            checker_type: CollisionCheckerType enum value (MESH, BLOX, or PRIMITIVE)
        """
        self.collision_checker_type = checker_type
        self.current_collision_checker = checker_type
        self._update_collision_cache()

        self.node.get_logger().info(
            f"Collision checker type set to: {checker_type}, "
            f"cache: {self.collision_cache}"
        )

    def get_all_obstacles_for_world_config(self):
        """
        Get all obstacles for world config update.
        Returns cuboid_list + mesh_list (or converted based on checker type).
        """
        # This method can be extended to handle mesh-to-cuboid conversion
        # when using BLOX checker, as done in ConfigWrapperMotion
        return self.cuboid_list + self.mesh_list
