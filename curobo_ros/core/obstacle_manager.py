import os
import numpy as np
import torch
import ros2_numpy as rnp

from std_srvs.srv import Trigger
from curobo_msgs.srv import AddObject, RemoveObject, GetVoxelGrid, SetCollisionCache
from geometry_msgs.msg import Vector3

from curobo.scene import Scene, Cuboid, Capsule, Cylinder, Sphere, Mesh


class ObstacleManager:
    """
    Manages runtime obstacles in a single cuRobo v2 `Scene`.

    v2 simplifications vs v1:
    - `WorldConfig` → `Scene`; all primitives live in a single flat container.
    - No more mesh ↔ cuboid dual-storage: a `Mesh` goes directly into the
      scene and is stamped into the Mapper's TSDF by the planner.
    - No more triple collision cache (obb / mesh / blox) — v2 uses a single
      `collision_cache` parameter on `MotionPlannerCfg`.
    - No more `MeshBloxilization` class: mesh voxelisation is handled
      natively by the Mapper.
    """

    def __init__(self, node, config_manager, initial_scene: Scene = None):
        self.node = node
        self.config_manager = config_manager

        self.scene = initial_scene if initial_scene is not None else Scene()
        self._ensure_scene_lists()

        # Tracks registered obstacle names for uniqueness validation.
        self.obstacle_names = []

        # Single v2 collision cache parameter (was obb/mesh/blox triple in v1).
        self.collision_cache = 100

        if initial_scene is not None:
            node.get_logger().info("ObstacleManager initialized with scene from ConfigManager")
        else:
            node.get_logger().info("ObstacleManager initialized with empty scene")

    # ---- Internal helpers ----

    def _ensure_scene_lists(self):
        """Make sure each primitive bucket on the Scene is a list (not None)."""
        for attr in ('cuboid', 'capsule', 'cylinder', 'sphere', 'mesh'):
            if getattr(self.scene, attr, None) is None:
                setattr(self.scene, attr, [])

    def _append(self, bucket: str, obstacle):
        getattr(self.scene, bucket).append(obstacle)
        self.obstacle_names.append(obstacle.name)

    # ---- Services ----

    def add_object(self, node, request: AddObject, response):
        """
        Add a primitive or mesh obstacle to the scene.

        Dimensions interpretation by type:
          CUBOID  -> [dx, dy, dz]
          CAPSULE -> [radius, height, _]
          CYLINDER-> [radius, height, _]
          SPHERE  -> [radius, _, _]
          MESH    -> [scale_x, scale_y, scale_z]
        """
        if request.name in self.obstacle_names:
            response.success = False
            response.message = f'Object with name "{request.name}" already exists'
            return response

        if request.dimensions.x <= 0 or request.dimensions.y <= 0 or request.dimensions.z <= 0:
            response.success = False
            response.message = 'Object dimensions must be positive'
            return response

        pose = [
            request.pose.position.x, request.pose.position.y, request.pose.position.z,
            request.pose.orientation.w, request.pose.orientation.x,
            request.pose.orientation.y, request.pose.orientation.z,
        ]
        dims = [request.dimensions.x, request.dimensions.y, request.dimensions.z]
        color = [request.color.r, request.color.g, request.color.b, request.color.a]

        try:
            match request.type:
                case request.CUBOID:
                    self._append('cuboid', Cuboid(name=request.name, pose=pose, dims=dims, color=color))

                case request.CAPSULE:
                    self._append('capsule', Capsule(
                        name=request.name, pose=pose,
                        base=[0, 0, 0], tip=[0, 0, dims[1]],
                        radius=dims[0], color=color,
                    ))

                case request.CYLINDER:
                    self._append('cylinder', Cylinder(
                        name=request.name, pose=pose,
                        radius=dims[0], height=dims[1], color=color,
                    ))

                case request.SPHERE:
                    self._append('sphere', Sphere(
                        name=request.name, pose=pose, radius=dims[0], color=color,
                    ))

                case request.MESH:
                    if not os.path.exists(request.mesh_file_path):
                        response.success = False
                        response.message = f'Mesh file not found: {request.mesh_file_path}'
                        return response
                    self._append('mesh', Mesh(
                        name=request.name, pose=pose,
                        file_path=request.mesh_file_path,
                        scale=dims, color=color,
                    ))
                    node.get_logger().info(
                        f"Added MESH obstacle '{request.name}' "
                        f"(handled natively by Mapper TSDF in v2)"
                    )

                case _:
                    response.success = False
                    response.message = f'Object type "{request.type}" not recognized'
                    return response

        except Exception as e:
            response.success = False
            response.message = f'Failed to add obstacle: {e}'
            node.get_logger().error(response.message)
            return response

        response.success = True
        response.message = (
            f"Object '{request.name}' added "
            f"({len(self.scene.cuboid)} cuboids, {len(self.scene.mesh)} meshes)"
        )
        return response

    def remove_object(self, node, request: RemoveObject, response):
        if request.name not in self.obstacle_names:
            response.success = False
            response.message = f"Object '{request.name}' not found"
            return response

        for bucket in ('cuboid', 'capsule', 'cylinder', 'sphere', 'mesh'):
            items = getattr(self.scene, bucket)
            for i, obs in enumerate(items):
                if obs.name == request.name:
                    items.pop(i)
                    self.obstacle_names.remove(request.name)
                    response.success = True
                    response.message = f"Object '{request.name}' removed from {bucket}s"
                    node.get_logger().info(response.message)
                    return response

        # Name was tracked but not found in any bucket — defensive cleanup.
        self.obstacle_names.remove(request.name)
        response.success = False
        response.message = f"Object '{request.name}' tracked but not in any scene bucket"
        return response

    def remove_all_objects(self, node, request: Trigger, response):
        total = sum(len(getattr(self.scene, b)) for b in ('cuboid', 'capsule', 'cylinder', 'sphere', 'mesh'))
        for bucket in ('cuboid', 'capsule', 'cylinder', 'sphere', 'mesh'):
            setattr(self.scene, bucket, [])
        self.obstacle_names = []

        response.success = True
        response.message = f'All {total} obstacles removed'
        node.get_logger().info(response.message)
        return response

    def get_obstacles(self, node, request: Trigger, response):
        response.message = "\n".join(self.obstacle_names) + ("\n" if self.obstacle_names else "")
        response.success = True
        return response

    def get_voxel_grid(self, node, request: GetVoxelGrid, response):
        """
        Return the ESDF voxel grid from the planner's Mapper, if available.

        The node is expected to expose `mapper` (curobo.perception.Mapper)
        once v2 setup is complete. If absent, an empty grid is returned.
        """
        voxel_size = node.get_parameter('voxel_size').get_parameter_value().double_value

        grid_min = np.array([-1.52, -1.52, -1.52], dtype=np.float32)
        grid_max = np.array([1.52, 1.52, 1.52], dtype=np.float32)
        size = np.ceil((grid_max - grid_min) / voxel_size).astype(np.int32)

        voxel_grid = np.zeros((size[0], size[1], size[2]), dtype=np.uint32)

        mapper = getattr(node, 'mapper', None)
        if mapper is not None:
            try:
                vg = mapper.compute_esdf()
                # v2 VoxelGrid exposes `data` as a dense tensor. Extract
                # occupied cells (sdf <= 0) and project into our grid.
                data = vg.data.detach().cpu().numpy() if torch.is_tensor(vg.data) else np.asarray(vg.data)
                origin = np.asarray(vg.origin, dtype=np.float32)
                vsize = float(getattr(vg, 'voxel_size', voxel_size))
                occ = np.argwhere(data <= 0.0)
                for ix, iy, iz in occ:
                    world_xyz = origin + np.array([ix, iy, iz]) * vsize
                    idx = np.floor((world_xyz - grid_min) / voxel_size).astype(np.int32)
                    if np.all((idx >= 0) & (idx < size)):
                        voxel_grid[tuple(idx)] = 1
                node.get_logger().info(
                    f'Filled voxel grid from Mapper ESDF ({len(occ)} occupied cells)'
                )
            except Exception as e:
                node.get_logger().warn(f'Mapper ESDF query failed: {e}')

        response.voxel_grid.resolutions = rnp.msgify(
            Vector3, np.array([voxel_size, voxel_size, voxel_size])
        )
        response.voxel_grid.size_x = int(size[0])
        response.voxel_grid.size_y = int(size[1])
        response.voxel_grid.size_z = int(size[2])
        response.voxel_grid.origin.x = float(grid_min[0])
        response.voxel_grid.origin.y = float(grid_min[1])
        response.voxel_grid.origin.z = float(grid_min[2])
        response.voxel_grid.data = voxel_grid.flatten().tolist()
        return response

    def set_collision_cache(
        self,
        node,
        request: SetCollisionCache.Request,
        response: SetCollisionCache.Response,
    ) -> SetCollisionCache.Response:
        """
        v2: a single `collision_cache` integer replaces the v1 obb/mesh/blox
        triple. We accept the maximum of the requested values for back-compat
        with clients still populating all three fields.
        """
        requested = [v for v in (request.obb, request.mesh, request.blox) if v >= 0]
        if requested:
            self.collision_cache = int(max(requested))
            node.get_logger().info(f'collision_cache set to {self.collision_cache}')
            node.get_logger().warn(
                'Collision cache changed — call /update_motion_gen_config to re-warmup.'
            )

        response.success = True
        response.message = 'Collision cache updated'
        response.obb_cache = self.collision_cache
        response.mesh_cache = self.collision_cache
        response.blox_cache = self.collision_cache
        return response

    # ---- Getters ----

    def get_scene(self) -> Scene:
        """Return the authoritative Scene (single source of truth)."""
        return self.scene

    # Legacy alias — some callers still use get_world_cfg().
    def get_world_cfg(self) -> Scene:
        return self.scene

    def get_all_obstacles(self):
        return (
            self.scene.cuboid
            + self.scene.capsule
            + self.scene.cylinder
            + self.scene.sphere
            + self.scene.mesh
        )

    def get_object(self, object_name: str) -> dict:
        for bucket in ('cuboid', 'capsule', 'cylinder', 'sphere', 'mesh'):
            for obj in getattr(self.scene, bucket):
                if obj.name == object_name:
                    return {'type': bucket, 'pose': obj.pose, 'object': obj}
        raise ValueError(f"Object '{object_name}' not found in obstacle manager")
