import os
import math
import numpy as np
import torch
import trimesh
import trimesh.voxel.creation as vox_creation
from scipy.ndimage import binary_fill_holes
import ros2_numpy as rnp
from std_srvs.srv import Trigger
from curobo_msgs.srv import AddObject, RemoveObject, GetVoxelGrid, SetCollisionCache
from curobo.geom.types import Cuboid, Capsule, Cylinder, Sphere, Mesh
from curobo.geom.sdf.world import CollisionCheckerType
from geometry_msgs.msg import Vector3


class ObstacleManager:
    """
    Manages obstacles and voxel grid computation.
    Responsible for:
    - Managing obstacle lists (cuboid_list, mesh_list, obstacle_names)
    - Adding/removing obstacles with validation
    - Converting primitive geometries to cuboids (including meshes)
    - Computing voxel grids for collision checking

    Mesh Handling Strategy:
    - Meshes are stored in BOTH world_cfg.mesh and world_cfg.cuboid lists
    - MESH checker: Uses precise mesh geometry from world_cfg.mesh
    - BLOX checker: Uses voxelized cuboid approximations from world_cfg.cuboid
    - Voxelization: MeshBloxilization converts mesh to multiple cuboids
    - Tracking: mesh_cuboid_mapping tracks derived cuboids for cleanup
    - This dual storage enables seamless runtime collision checker switching
    """

    def __init__(self, node, config_manager, initial_world_cfg: 'WorldConfig' = None):
        """
        Initialize obstacle manager.

        Args:
            node: ROS2 node instance
            config_manager: ConfigManager for accessing configuration
            initial_world_cfg: Optional initial WorldConfig from ConfigManager
        """
        from curobo.geom.types import WorldConfig

        self.node = node
        self.config_manager = config_manager

        # Initialize world_cfg - either from initial config or create empty
        if initial_world_cfg is not None:
            # Use provided world_cfg as base, preserving any blox/voxel settings
            self.world_cfg = initial_world_cfg
            # Ensure obstacle lists are initialized
            if self.world_cfg.cuboid is None:
                self.world_cfg.cuboid = []
            if self.world_cfg.mesh is None:
                self.world_cfg.mesh = []
            if self.world_cfg.capsule is None:
                self.world_cfg.capsule = []
            if self.world_cfg.cylinder is None:
                self.world_cfg.cylinder = []
            if self.world_cfg.sphere is None:
                self.world_cfg.sphere = []
            node.get_logger().info(
                "ObstacleManager initialized with world_cfg from ConfigManager"
            )
        else:
            # Create empty world_cfg (backward compatibility)
            self.world_cfg = WorldConfig(
                mesh=[],
                cuboid=[],
                capsule=[],
                cylinder=[],
                sphere=[],
            )
            node.get_logger().info(
                "ObstacleManager initialized with empty world_cfg"
            )

        # Track all obstacle names for uniqueness validation
        self.obstacle_names = []

        # Track mesh-to-cuboid mappings for cleanup
        # Format: {mesh_name: [cuboid_name_0, cuboid_name_1, ...]}
        self.mesh_cuboid_mapping = {}

        # Default voxel size for mesh voxelization (configurable)
        self.mesh_voxel_size = 0.05  # 2cm par défaut

        # Collision checker configuration
        # Default collision checker (can be changed dynamically)
        self.collision_checker_type = CollisionCheckerType.BLOX


        # Adaptive collision cache based on checker type
        self.collision_cache = {'obb': 100, 'mesh': 10, 'blox': 10}

    def add_object(self, node, request: AddObject, response):
        """
        Add an object to the world configuration.
        The object is created based on the type, pose, dimensions and color requested.

        Mesh Handling:
        - Mesh obstacles are voxelized using MeshBloxilization class
        - Result: One mesh + N cuboids (stored in separate lists)
        - MESH checker: Uses precise mesh geometry from world_cfg.mesh
        - BLOX checker: Uses voxelized cuboid approximations from world_cfg.cuboid
        - All derived cuboids have names: {mesh_name}_cuboid_{idx}
        - Mapping stored in mesh_cuboid_mapping for cleanup on removal

        Other Primitives (Capsule, Cylinder, Sphere):
        - Converted to cuboids using .get_cuboid() method
        - Stored only in world_cfg.cuboid list
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
                self.world_cfg.cuboid.append(obstacle)
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
                self.world_cfg.cuboid.append(obstacle)
                self.obstacle_names.append(request.name)

            case request.CYLINDER:
                obstacle = Cylinder(
                    name=request.name,
                    pose=extracted_pose,
                    radius=extracted_dimensions[0],
                    height=extracted_dimensions[1],
                    color=extracted_color,
                ).get_cuboid()
                self.world_cfg.cuboid.append(obstacle)
                self.obstacle_names.append(request.name)

            case request.SPHERE:
                obstacle = Sphere(
                    name=request.name,
                    pose=extracted_pose,
                    radius=extracted_dimensions[0],
                    color=extracted_color,
                ).get_cuboid()
                self.world_cfg.cuboid.append(obstacle)
                self.obstacle_names.append(request.name)

            case request.MESH:
                # Check file exists
                if not os.path.exists(request.mesh_file_path):
                    response.success = False
                    response.message = f"Mesh file not found: {request.mesh_file_path}"
                    return response

                try:
                    # 1. Voxelize first (before touching any state) to know the cuboid count
                    node.get_logger().info(
                        f"Voxelizing mesh '{request.name}' (voxel_size={self.mesh_voxel_size})..."
                    )
                    blox = MeshBloxilization(
                        mesh_path=request.mesh_file_path,
                        voxel_size=self.mesh_voxel_size,
                        logger=node.get_logger()
                    )
                    cubes = blox.pipeline(
                        scale=extracted_dimensions,
                        pose=extracted_pose,
                    )

                    # 2. Pre-check: refuse if adding these cuboids would exceed the OBB cache
                    current_cuboids = len(self.world_cfg.cuboid)
                    obb_cache = self.collision_cache['obb']
                    if current_cuboids + len(cubes) > obb_cache:
                        response.success = False
                        response.message = (
                            f"OBB cache too small: adding {len(cubes)} cuboids would reach "
                            f"{current_cuboids + len(cubes)} > cache {obb_cache}. "
                            f"Use /set_collision_cache to increase it."
                        )
                        node.get_logger().error(response.message)
                        return response

                    # 3. Add Mesh object for MESH collision checker
                    mesh_obstacle = Mesh(
                        name=request.name,
                        pose=extracted_pose,
                        file_path=request.mesh_file_path,
                        scale=extracted_dimensions,
                        color=extracted_color,
                    )
                    self.world_cfg.mesh.append(mesh_obstacle)

                    # 4. Create cuboids from merged cubes
                    cuboid_names = []
                    for idx, (origin, dims) in enumerate(cubes):
                        cuboid_name = f"{request.name}_cuboid_{idx}"
                        center = origin + dims / 2.0
                        cuboid_pose = list(center) + [1.0, 0.0, 0.0, 0.0]
                        cuboid = Cuboid(
                            name=cuboid_name,
                            pose=cuboid_pose,
                            dims=list(dims),
                            color=extracted_color,
                        )
                        self.world_cfg.cuboid.append(cuboid)
                        cuboid_names.append(cuboid_name)
                        self.obstacle_names.append(cuboid_name)

                    # 5. Save mapping and register name
                    self.mesh_cuboid_mapping[request.name] = cuboid_names
                    self.obstacle_names.append(request.name)

                    node.get_logger().info(
                        f"Added MESH obstacle '{request.name}' "
                        f"(mesh + {len(cubes)} cuboids, checker: {self.collision_checker_type})"
                    )

                except Exception as e:
                    response.success = False
                    response.message = f"Failed to load/voxelize mesh: {str(e)}"
                    node.get_logger().error(f"Mesh processing error: {str(e)}")
                    return response

            case _:  # default
                response.success = False
                response.message = 'Object type "' + str(request.type) + '" not recognized'

        if response.success:
            response.message = 'Object ' + request.name + ' added successfully'

        return response

    def remove_object(self, node, request: RemoveObject, response):
        """
        Remove an object from the world configuration.

        For mesh obstacles: removes both the mesh AND all derived cuboids.
        For other obstacles: removes from the appropriate list.
        """
        found = False
        removed_types = []

        # 1. If it's a mesh with derived cuboids, remove all cuboids first
        if request.name in self.mesh_cuboid_mapping:
            cuboid_names = self.mesh_cuboid_mapping[request.name]

            # Remove all derived cuboids
            for cuboid_name in cuboid_names:
                # Remove from world_cfg.cuboid
                for i, obs in enumerate(self.world_cfg.cuboid):
                    if obs.name == cuboid_name:
                        self.world_cfg.cuboid.pop(i)
                        break

                # Remove from obstacle_names
                if cuboid_name in self.obstacle_names:
                    self.obstacle_names.remove(cuboid_name)

            # Clean up the mapping
            del self.mesh_cuboid_mapping[request.name]
            removed_types.append(f"{len(cuboid_names)} cuboids")
            node.get_logger().info(
                f"Removed {len(cuboid_names)} derived cuboids for mesh '{request.name}'"
            )

        # 2. Search and remove from mesh list
        for i, obs in enumerate(self.world_cfg.mesh):
            if obs.name == request.name:
                self.world_cfg.mesh.pop(i)
                found = True
                removed_types.append("mesh")
                break

        # 3. If not found in mesh, search in cuboid (for non-mesh obstacles)
        if not found:
            for i, obs in enumerate(self.world_cfg.cuboid):
                if obs.name == request.name:
                    self.world_cfg.cuboid.pop(i)
                    found = True
                    removed_types.append("cuboid")
                    break

        if not found:
            response.success = False
            response.message = f"Object '{request.name}' not found"
            return response

        try:
            # Remove the main name from the list
            if request.name in self.obstacle_names:
                self.obstacle_names.remove(request.name)

            response.success = True
            types_str = "+".join(removed_types)
            response.message = f"Object '{request.name}' removed successfully ({types_str})"

            node.get_logger().info(f"Removed obstacle '{request.name}' ({types_str})")

        except Exception as e:
            response.success = False
            response.message = f"Failed to remove '{request.name}': {str(e)}"
            return response

        return response

    def remove_all_objects(self, node, request: Trigger, response):
        """
        Remove all objects from the world configuration.
        Clears both cuboid and mesh obstacles from self.world_cfg.
        Also clears mesh-to-cuboid mappings.
        """
        # Clear all obstacles from the world configuration
        num_cuboids = len(self.world_cfg.cuboid)
        num_meshes = len(self.world_cfg.mesh)

        self.world_cfg.cuboid = []
        self.world_cfg.mesh = []
        self.obstacle_names = []
        self.mesh_cuboid_mapping = {}  # Clear mesh-cuboid mappings

        response.success = True
        response.message = f'All objects removed successfully ({num_cuboids} cuboids, {num_meshes} meshes)'

        node.get_logger().info(
            f"Cleared all obstacles: {num_cuboids} cuboids, {num_meshes} meshes"
        )

        return response

    def get_obstacles(self, node, request: Trigger, response):
        """
        Get list of all obstacle names from ObstacleManager's world_cfg.

        Returns:
            Trigger.Response with message containing all obstacle names
        """
        for name in self.obstacle_names:
            response.message = response.message + name + "\n"

        response.success = True
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


    def set_collision_cache(self, node, request: SetCollisionCache.Request, response: SetCollisionCache.Response):
        """
        Service handler to modify collision cache parameters dynamically.

        Args:
            node: ROS2 node instance
            request: SetCollisionCache request with obb, mesh, blox values (-1 = don't modify)
            response: SetCollisionCache response

        Returns:
            response with success status and current cache values
        """
        try:
            # Update cache values (only if not -1)
            if request.obb >= 0:
                self.collision_cache['obb'] = request.obb
                node.get_logger().info(f"Updated OBB cache to: {request.obb}")

            if request.mesh >= 0:
                self.collision_cache['mesh'] = request.mesh
                node.get_logger().info(f"Updated mesh cache to: {request.mesh}")

            if request.blox >= 0:
                self.collision_cache['blox'] = request.blox
                node.get_logger().info(f"Updated blox cache to: {request.blox}")

            # Set response
            response.success = True
            response.message = "Collision cache updated successfully"
            response.obb_cache = self.collision_cache['obb']
            response.mesh_cache = self.collision_cache['mesh']
            response.blox_cache = self.collision_cache['blox']

            node.get_logger().info(
                f"Collision cache: obb={response.obb_cache}, "
                f"mesh={response.mesh_cache}, blox={response.blox_cache}"
            )
            node.get_logger().warn(
                f"Collision cache changed — a warmup is required. "
                f"Call: ros2 service call /{node.get_name()}/update_motion_gen_config std_srvs/srv/Trigger"
            )

        except Exception as e:
            response.success = False
            response.message = f"Failed to update collision cache: {str(e)}"
            response.obb_cache = self.collision_cache.get('obb', 0)
            response.mesh_cache = self.collision_cache.get('mesh', 0)
            response.blox_cache = self.collision_cache.get('blox', 0)
            node.get_logger().error(f"Error updating collision cache: {str(e)}")

        return response

    def get_all_obstacles_for_world_config(self):
        """
        Get all obstacles for world config update.
        Returns all obstacles from world_cfg (cuboid + mesh).
        """
        # Return all obstacles from the world configuration
        return self.world_cfg.cuboid + self.world_cfg.mesh

    def get_world_cfg(self):
        """
        Get the current WorldConfig (single source of truth).

        This is the authoritative world configuration that includes all obstacles.
        All other classes should use this getter instead of direct access.

        Returns:
            WorldConfig: The current world configuration with all obstacles

        Thread Safety:
            Returns the reference to the internal world_cfg. Python's GIL provides
            sufficient protection for read operations. Modifications should only
            happen through ObstacleManager's methods (add_object, remove_object, etc.)
        """
        return self.world_cfg

    def update_voxel_size(self, voxel_size: float):
        """
        Update voxel size in world_cfg.blox configuration.

        Args:
            voxel_size: New voxel size in meters
        """
        if self.world_cfg.blox is not None and len(self.world_cfg.blox) > 0:
            self.world_cfg.blox[0].voxel_size = voxel_size
            self.node.get_logger().info(f"Updated voxel size to {voxel_size}m")

    def get_object(self, object_name: str) -> dict:
        """
        Get object information by name.

        Returns dictionary with object information:
            - 'type': 'cuboid' | 'cylinder' | 'sphere' | 'capsule' | 'mesh'
            - 'pose': pose as [x, y, z, qw, qx, qy, qz] list
            - 'dimensions': size parameters (depends on type)
            - 'object': original object reference

        Args:
            object_name: Name of the object to retrieve

        Returns:
            dict with object information

        Raises:
            ValueError: If object not found
        """
        # Search in cuboid list
        for obj in self.world_cfg.cuboid:
            if obj.name == object_name:
                return {
                    'type': 'cuboid',
                    'pose': obj.pose,  # [x, y, z, qw, qx, qy, qz]
                    'dimensions': {
                        'dims': obj.dims  # [width, length, height]
                    },
                    'object': obj
                }

        # Search in cylinder list
        for obj in self.world_cfg.cylinder:
            if obj.name == object_name:
                return {
                    'type': 'cylinder',
                    'pose': obj.pose,
                    'dimensions': {
                        'radius': obj.radius,
                        'height': obj.height
                    },
                    'object': obj
                }

        # Search in sphere list
        for obj in self.world_cfg.sphere:
            if obj.name == object_name:
                return {
                    'type': 'sphere',
                    'pose': obj.pose,
                    'dimensions': {
                        'radius': obj.radius
                    },
                    'object': obj
                }

        # Search in capsule list
        for obj in self.world_cfg.capsule:
            if obj.name == object_name:
                return {
                    'type': 'capsule',
                    'pose': obj.pose,
                    'dimensions': {
                        'radius': obj.radius,
                        'base': obj.base,
                        'tip': obj.tip
                    },
                    'object': obj
                }

        # Search in mesh list
        for obj in self.world_cfg.mesh:
            if obj.name == object_name:
                return {
                    'type': 'mesh',
                    'pose': obj.pose,
                    'dimensions': {
                        'scale': obj.scale,
                        'file_path': obj.file_path
                    },
                    'object': obj
                }

        # Not found in any list
        raise ValueError(f"Object '{object_name}' not found in obstacle manager")



class MeshBloxilization():
    """
    Converts a mesh into a list of axis-aligned cuboids for cuRobo's BLOX collision checker.

    Pipeline:
      1. Load mesh, apply scale + pose
      2. Surface voxelization (trimesh) → shell occupancy grid
      3. binary_fill_holes → solid occupancy grid
      4. Greedy 3D merge → fuse adjacent voxels into larger cuboids
    """

    def __init__(self, mesh_path, voxel_size, logger=None):
        self.mesh_path = mesh_path
        self.voxel_size = voxel_size
        self.logger = logger

    def _log(self, msg):
        if self.logger:
            self.logger.info(msg)
        else:
            print(msg)

    def pipeline(self, scale=None, pose=None, visualize=False, export_path=None):
        """
        Voxelize a mesh and merge adjacent voxels into larger cuboids.

        Args:
            scale: Optional [sx, sy, sz] scaling factors
            pose: Optional [x, y, z, qw, qx, qy, qz] pose in world frame
            visualize: Show result in trimesh viewer
            export_path: Optional path to export the voxel mesh

        Returns:
            List of (origin, dims) tuples where origin and dims are numpy arrays (shape (3,)).
            Cuboids may have non-uniform dims when neighbouring voxels are merged.
        """
        mesh = trimesh.load(self.mesh_path, force='mesh')
        if mesh.is_empty:
            raise RuntimeError(f"Mesh empty or not loaded: {self.mesh_path}")

        if scale is not None:
            mesh.apply_scale(scale)

        if pose is not None:
            T = trimesh.transformations.quaternion_matrix([pose[3], pose[4], pose[5], pose[6]])
            T[:3, 3] = pose[:3]
            mesh.apply_transform(T)

        self._log(f"Voxelizing mesh (pitch={self.voxel_size}m) ...")
        vox = vox_creation.voxelize(mesh, pitch=self.voxel_size)

        # Fill interior: surface voxelization produces a shell, this makes it solid
        filled_vox = trimesh.voxel.VoxelGrid(binary_fill_holes(vox.matrix), vox.transform)

        # Greedy merging: fuse adjacent voxels into larger cuboids
        half = self.voxel_size / 2.0
        grid_origin = np.array(filled_vox.transform[:3, 3]) - half  # world-space corner of voxel [0,0,0]
        raw_count = int(np.sum(filled_vox.matrix))
        cubes = self._merge_cubes(filled_vox.matrix, grid_origin, self.voxel_size)
        self._log(f"Merged {raw_count} voxels → {len(cubes)} cuboids "
                  f"(ratio {raw_count / max(len(cubes), 1):.1f}x)")

        if visualize:
            trimesh.Scene([mesh, filled_vox.as_boxes()]).show()

        if export_path:
            filled_vox.as_boxes().export(export_path)
            self._log(f"Exported voxel mesh to {export_path}")

        return cubes

    def _merge_cubes(self, voxel_grid, origin, voxel_size):
        """
        Greedy 3D merging: fuses adjacent occupied voxels into larger axis-aligned cuboids.

        Single pass in X→Y→Z order. For each unprocessed voxel, extend as far as possible
        along X, then Y (full X slab), then Z (full XY slab). Mark the region as consumed.

        Args:
            voxel_grid: 3D boolean numpy array (solid occupancy grid)
            origin: world-space corner of voxel at index [0,0,0], shape (3,)
            voxel_size: size of one voxel in meters

        Returns:
            List of (cube_origin, cube_dims) — both numpy arrays, shape (3,).
            cube_dims >= voxel_size on each axis.
        """
        grid = voxel_grid.copy()
        nx, ny, nz = grid.shape
        cuboids = []

        for x in range(nx):
            for y in range(ny):
                for z in range(nz):
                    if not grid[x, y, z]:
                        continue

                    # Extend along X
                    dx = 1
                    while x + dx < nx and grid[x + dx, y, z]:
                        dx += 1

                    # Extend along Y (entire X slab must be occupied)
                    dy = 1
                    while y + dy < ny and np.all(grid[x:x + dx, y + dy, z]):
                        dy += 1

                    # Extend along Z (entire XY slab must be occupied)
                    dz = 1
                    while z + dz < nz and np.all(grid[x:x + dx, y:y + dy, z + dz]):
                        dz += 1

                    # Consume merged region
                    grid[x:x + dx, y:y + dy, z:z + dz] = False

                    cube_origin = origin + np.array([x, y, z]) * voxel_size
                    cube_dims = np.array([dx, dy, dz], dtype=float) * voxel_size
                    cuboids.append((cube_origin, cube_dims))

        return cuboids