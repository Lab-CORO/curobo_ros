import os
import math
import numpy as np
import torch
import open3d as o3d
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
                    # 1. Create Mesh object for MESH collision checker
                    # Preserves precise mesh geometry for high-fidelity collision checking
                    mesh_obstacle = Mesh(
                        name=request.name,
                        pose=extracted_pose,
                        file_path=request.mesh_file_path,
                        scale=extracted_dimensions,
                        color=extracted_color,
                    )
                    self.world_cfg.mesh.append(mesh_obstacle)

                    # 2. Voxelize the mesh for BLOX collision checker
                    # Apply scale and pose transformations before voxelization
                    node.get_logger().info(
                        f"Voxelizing mesh '{request.name}' (voxel_size={self.mesh_voxel_size})..."
                    )
                    cubes = MeshBloxilization.pipeline(
                        request.mesh_file_path,
                        voxel_size=self.mesh_voxel_size,
                        scale=extracted_dimensions,  # Apply scale transformation
                        pose=extracted_pose,          # Apply pose transformation
                        nsamples=1,
                        visualize=False,
                        export_path=None
                    )

                    # 3. Create cuboids from voxelized cubes
                    # Cubes are already in world coordinates thanks to transformations in pipeline()
                    cuboid_names = []
                    for idx, (origin, size) in enumerate(cubes):
                        cuboid_name = f"{request.name}_cuboid_{idx}"

                        # origin is the cube corner (in world coordinates), compute center
                        center = origin + np.array([size/2, size/2, size/2])

                        # Cuboids are axis-aligned in world frame (no rotation)
                        # The mesh was rotated before voxelization, so cuboids are correctly positioned
                        cuboid_pose = list(center) + [1.0, 0.0, 0.0, 0.0]  # [x,y,z,qw,qx,qy,qz]

                        cuboid = Cuboid(
                            name=cuboid_name,
                            pose=cuboid_pose,
                            dims=[size, size, size],
                            color=extracted_color,
                        )
                        self.world_cfg.cuboid.append(cuboid)
                        cuboid_names.append(cuboid_name)
                        self.obstacle_names.append(cuboid_name)

                    # 4. Save mapping for deletion
                    self.mesh_cuboid_mapping[request.name] = cuboid_names

                    # 5. Add mesh name to obstacle_names
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



class MeshBloxilization():
    """
    This class is temporary to convert mesh to cuboid.
    Later the build-in solution from curobo will be used
    """

    def __init__(self, mesh_path, voxel_size):
        self.mesh_path = mesh_path
        self.voxel_size = voxel_size

    @staticmethod
    def ensure_power_of_two_dims(shape):
        # pad shape to next power-of-two in each dimension
        return [1 << math.ceil(math.log2(s)) for s in shape]

    @staticmethod
    def build_dense_grid(mesh, voxel_size):
        # bbox and grid shape (integer number of voxels to cover bbox)
        bbox_min = mesh.get_min_bound()
        bbox_max = mesh.get_max_bound()
        extent = bbox_max - bbox_min
        # number of voxels along each axis (ceil to cover full bbox)
        Nx = int(np.ceil(extent[0] / voxel_size))
        Ny = int(np.ceil(extent[1] / voxel_size))
        Nz = int(np.ceil(extent[2] / voxel_size))

        # pad to power-of-two sizes for merging convenience
        Nx_p, Ny_p, Nz_p = MeshBloxilization.ensure_power_of_two_dims((Nx, Ny, Nz))
        pad_x = Nx_p - Nx
        pad_y = Ny_p - Ny
        pad_z = Nz_p - Nz

        print(f"Grid base (Nx,Ny,Nz) = {(Nx,Ny,Nz)}, padded -> {(Nx_p,Ny_p,Nz_p)}")

        # compute centers for original (unpadded) grid, but we will map into padded array
        # create coordinates for padded grid such that origin remains bbox_min
        xs = (np.arange(Nx_p) + 0.5) * voxel_size + bbox_min[0]
        ys = (np.arange(Ny_p) + 0.5) * voxel_size + bbox_min[1]
        zs = (np.arange(Nz_p) + 0.5) * voxel_size + bbox_min[2]

        # create array of centers in (N,3)
        # to save memory, we generate flattened centers and query occupancy in batches
        return {
            "bbox_min": bbox_min,
            "bbox_max": bbox_max,
            "voxel_size": voxel_size,
            "grid_shape": (Nx_p, Ny_p, Nz_p),
            "orig_grid_shape": (Nx, Ny, Nz),
            "coords_arrays": (xs, ys, zs),
            "pad": (pad_x, pad_y, pad_z)
        }

    @staticmethod
    def compute_occupancy(mesh, grid_meta, batch_size=200000, nsamples=1):
        # Use RaycastingScene to compute occupancy at voxel centers.
        # Convert legacy mesh to t.geometry
        tmesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
        scene = o3d.t.geometry.RaycastingScene()
        scene.add_triangles(tmesh)

        xs, ys, zs = grid_meta["coords_arrays"]
        Nx, Ny, Nz = grid_meta["grid_shape"]

        # We'll iterate over z slices to reduce memory pressure
        occ = np.zeros((Nx, Ny, Nz), dtype=bool)

        total = Nx * Ny * Nz
        print(f"Computing occupancy for {total} voxels (in slices) ...")

        # Prepare all centers per z slice
        for k, z in enumerate(zs):
            # create meshgrid for x,y for this z slice
            xx, yy = np.meshgrid(xs, ys, indexing="ij")  # shape (Nx,Ny)
            pts = np.stack([xx.ravel(), yy.ravel(), np.full(xx.size, z)], axis=-1)  # (Nx*Ny,3)
            # convert to tensor (float32)
            q = o3d.core.Tensor(pts.astype(np.float32))
            occ_slice = scene.compute_occupancy(q, nsamples=nsamples).numpy().astype(bool)  # (Nx*Ny,)
            occ[:, :, k] = occ_slice.reshape((Nx, Ny))
            if (k+1) % 10 == 0 or (k+1)==Nz:
                print(f"  slice {k+1}/{Nz} done")

        return occ

    @staticmethod
    def merge_level(prev):
        # prev: boolean numpy array shape (Nx,Ny,Nz)
        Nx, Ny, Nz = prev.shape
        # require even dims
        assert Nx % 2 == 0 and Ny % 2 == 0 and Nz % 2 == 0
        Nx2, Ny2, Nz2 = Nx//2, Ny//2, Nz//2
        merged = np.zeros((Nx2, Ny2, Nz2), dtype=bool)

        # vectorized approach using reshape and all over 2x2x2 blocks
        # reshape to (Nx2, 2, Ny2, 2, Nz2, 2) then all over axes 1,3,5
        m = prev.reshape(Nx2, 2, Ny2, 2, Nz2, 2)
        merged = m.all(axis=(1,3,5))
        return merged

    @staticmethod
    def build_levels(occ_base):
        levels = [occ_base]
        while True:
            prev = levels[-1]
            if prev.shape[0] % 2 != 0 or prev.shape[1] % 2 != 0 or prev.shape[2] % 2 != 0:
                break
            merged = MeshBloxilization.merge_level(prev)
            if not merged.any():
                break
            levels.append(merged)
        print(f"Built {len(levels)} levels (0..{len(levels)-1})")
        return levels

    @staticmethod
    def extract_cubes(levels, grid_meta):
        # Greedy extraction from largest to smallest to avoid overlaps
        base_shape = levels[0].shape
        Nx_base, Ny_base, Nz_base = base_shape
        voxel_size = grid_meta["voxel_size"]
        bbox_min = grid_meta["bbox_min"]

        coverage = np.zeros(base_shape, dtype=bool)
        cubes = []  # elements: (origin_xyz, size)

        max_lvl = len(levels) - 1
        for lvl in range(max_lvl, -1, -1):
            grid = levels[lvl]
            factor = 2 ** lvl
            size = voxel_size * factor
            Nx, Ny, Nz = grid.shape
            it = np.ndindex(Nx, Ny, Nz)
            for (i,j,k) in it:
                if not grid[i,j,k]:
                    continue
                # compute corresponding base indices range
                si = i * factor
                sj = j * factor
                sk = k * factor
                ei = si + factor
                ej = sj + factor
                ek = sk + factor
                # if already covered by a larger cube, skip
                if coverage[si:ei, sj:ej, sk:ek].all():
                    continue
                # accept this cube
                origin = bbox_min + np.array([si, sj, sk]) * voxel_size  # corner
                cubes.append((origin, size))
                # mark coverage
                coverage[si:ei, sj:ej, sk:ek] = True
        print(f"Extracted {len(cubes)} cubes")
        return cubes

    @staticmethod
    def cubes_to_meshes(cubes, color=(0.7,0.7,0.7)):
        meshes = []
        for origin, size in cubes:
            box = o3d.geometry.TriangleMesh.create_box(width=size, height=size, depth=size)
            box.compute_vertex_normals()
            # create_box is at [0,0,0]..[1,1,1]*size, so translate to origin
            box.translate(origin)
            box.paint_uniform_color(color)
            meshes.append(box)
        return meshes

    @staticmethod
    def export_combined_mesh(meshes, out_path):
        if len(meshes) == 0:
            print("No meshes to export")
            return
        combined = meshes[0]
        for m in meshes[1:]:
            combined += m
        print(f"Writing combined mesh to {out_path} ...")
        o3d.io.write_triangle_mesh(out_path, combined)

    @staticmethod
    def quat_to_rotation_matrix(quat):
        """
        Convert quaternion [qw, qx, qy, qz] to 3x3 rotation matrix.

        Args:
            quat: Quaternion as [qw, qx, qy, qz] (scalar-first convention)

        Returns:
            3x3 rotation matrix (numpy array)
        """
        qw, qx, qy, qz = quat

        # Rotation matrix from quaternion (standard formula)
        R = np.array([
            [1 - 2*(qy**2 + qz**2),     2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)],
            [    2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2),     2*(qy*qz - qw*qx)],
            [    2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
        ])

        return R

    # -----------------------------
    # Pipeline main
    # -----------------------------
    @staticmethod
    def pipeline(mesh_path, voxel_size=0.05, scale=None, pose=None, nsamples=1, visualize=False, export_path=None):
        """
        Voxelize a mesh into cuboids with optional scaling and pose transformation.

        Transformations are applied to the mesh using Open3D before voxelization,
        so the resulting cubes are already in world coordinates.

        Args:
            mesh_path: Path to mesh file
            voxel_size: Voxel size for discretization
            scale: Optional [sx, sy, sz] scaling factors to apply to mesh
            pose: Optional [x,y,z,qw,qx,qy,qz] pose in world frame
            nsamples: Number of samples for occupancy testing
            visualize: Whether to visualize result
            export_path: Optional path to export combined mesh

        Returns:
            List of (origin, size) tuples in world coordinates
        """
        mesh = o3d.io.read_triangle_mesh(mesh_path)
        if mesh.is_empty():
            raise RuntimeError("Mesh empty or not loaded")

        mesh.compute_vertex_normals()

        # Apply transformations using Open3D (before voxelization!)
        if scale is not None or pose is not None:
            # 1. Apply scaling
            if scale is not None:
                # Non-uniform scaling by modifying vertices directly
                vertices = np.asarray(mesh.vertices)
                vertices_scaled = vertices * np.array(scale)
                mesh.vertices = o3d.utility.Vector3dVector(vertices_scaled)

            # 2. Apply rotation and translation
            if pose is not None:
                translation = np.array(pose[:3])
                quat = pose[3:]  # [qw, qx, qy, qz]

                # Convert quaternion to rotation matrix
                rot_matrix = MeshBloxilization.quat_to_rotation_matrix(quat)

                # Apply rotation around mesh center, then translate
                mesh.rotate(rot_matrix, center=mesh.get_center())
                mesh.translate(translation)

        # Voxelize the transformed mesh
        # The cubes will already be in world coordinates!
        grid_meta = MeshBloxilization.build_dense_grid(mesh, voxel_size)

        # compute occupancy using RaycastingScene
        occ = MeshBloxilization.compute_occupancy(mesh, grid_meta, nsamples=nsamples)

        # build merging levels
        levels = MeshBloxilization.build_levels(occ)

        # extract final cubes
        cubes = MeshBloxilization.extract_cubes(levels, grid_meta)

        # create meshes for visualization
        if visualize or export_path:
            cube_meshes = MeshBloxilization.cubes_to_meshes(cubes)

        if visualize:
            # show original mesh lightly and cubes semi-transparent
            mesh_for_vis = mesh.paint_uniform_color([0.8,0.8,0.8])
            to_draw = [mesh] + cube_meshes
            o3d.visualization.draw_geometries(to_draw)

        if export_path:
            ext = os.path.splitext(export_path)[1].lower()
            # combine and export as one mesh
            MeshBloxilization.export_combined_mesh(cube_meshes, export_path)
            print("Export done")

        # also return cubes list for further processing
        return cubes