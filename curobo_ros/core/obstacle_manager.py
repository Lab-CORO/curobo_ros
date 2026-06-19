import os
import numpy as np
import torch
import ros2_numpy as rnp

from std_srvs.srv import Trigger
from curobo_msgs.srv import AddObject, RemoveObject, GetVoxelGrid, SetCollisionCache
from geometry_msgs.msg import Vector3

from curobo.scene import Scene, Cuboid, Capsule, Cylinder, Sphere, Mesh
from curobo.perception import Mapper, MapperCfg


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

        # Perception (created lazily in setup_perception()).
        self.mapper = None
        self._esdf_voxel_name = None

        # Observer callbacks (registered by ConfigWrapper). They decouple scene
        # mutations from solver propagation so that ANY caller — ROS service or
        # direct Python — reliably reaches the solvers.
        self._on_world_changed = None
        self._on_cache_changed = None

        # Perception/voxel parameters — single source of truth shared between
        # the Mapper (MapperCfg) and the solver voxel collision cache.
        self._load_perception_params()

        # v2 collision cache: dict with keys cuboid (int), mesh (int), voxel
        # (None or dict {"layers": int, "dims": [x,y,z], "voxel_size": float}).
        # v1 passed the triple obb/mesh/blox; the service still accepts it and
        # maps obb→cuboid, mesh→mesh, blox→voxel.layers.
        #
        # The voxel sub-dict pre-allocates voxel collision storage in the
        # solvers (VoxelData.create_cache) and MUST match the Mapper's ESDF
        # grid: dims = extent in meters, voxel_size = ESDF voxel size.
        self.collision_cache = {
            "cuboid": 100,
            "mesh": 100,
            "voxel": {
                "layers": 1,
                "dims": list(self._mapper_extent_xyz),
                "voxel_size": self._esdf_voxel_size,
            },
        }

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

    # ---- Perception parameters (single source of truth) ----

    def _declare_param(self, name, default):
        """Read a ROS param, declaring it with `default` if not yet present."""
        if not self.node.has_parameter(name):
            self.node.declare_parameter(name, default)
        return self.node.get_parameter(name).value

    def _load_perception_params(self):
        """Load voxel/perception params (declared with defaults if missing).

        The ESDF voxel size reuses the existing `voxel_size` param so a single
        knob drives MapperCfg.esdf_voxel_size, the voxel collision cache, and
        the get_voxel_grid resolution.
        """
        self._use_mapper = self._declare_param('use_mapper', True)
        self._mapper_extent_xyz = self._declare_param('mapper_extent_xyz', [2.0, 2.0, 2.0])
        self._mapper_grid_center = self._declare_param('mapper_grid_center', [0.0, 0.0, 0.0])
        self._mapper_voxel_size = self._declare_param('mapper_voxel_size', 0.02)
        self._mapper_image_height = self._declare_param('mapper_image_height', 480)
        self._mapper_image_width = self._declare_param('mapper_image_width', 640)
        self._mapper_depth_min = self._declare_param('mapper_depth_min', 0.1)
        self._mapper_depth_max = self._declare_param('mapper_depth_max', 5.0)
        # ESDF voxel size shares the pre-existing `voxel_size` param.
        self._esdf_voxel_size = self._declare_param('voxel_size', 0.05)

    # ---- Observer wiring (propagation to solvers) ----

    def set_world_update_callback(self, callback):
        """Register a callback invoked after every scene mutation."""
        self._on_world_changed = callback

    def set_collision_cache_callback(self, callback):
        """Register a callback invoked when the collision cache changes.

        A cache change requires a full solver rebuild (cache size is fixed at
        solver creation), not just a world update.
        """
        self._on_cache_changed = callback

    def _notify_world_changed(self):
        if self._on_world_changed is not None:
            self._on_world_changed()

    def _notify_cache_changed(self):
        if self._on_cache_changed is not None:
            self._on_cache_changed()

    # ---- Perception (Mapper / ESDF) ----

    def setup_perception(self, num_cameras: int = 1):
        """Create the v2 Mapper and expose it as `node.mapper`.

        Idempotent: if a Mapper already exists on the node (e.g. created by
        another ConfigWrapper sharing this node), adopt it instead of building
        a second one. No-op when `use_mapper` is False.

        Args:
            num_cameras: number of cameras feeding the shared Mapper (sizes the
                projective scratch buffer).
        """
        if not self._use_mapper:
            self.mapper = None
            self.node.mapper = None
            self.node.get_logger().info("Perception disabled (use_mapper=False)")
            return

        if int(num_cameras) < 1:
            # No camera feeds the Mapper → skip the GPU allocation entirely.
            self.mapper = None
            self.node.mapper = None
            self.node.get_logger().info("Perception inactive (no cameras configured)")
            return

        existing = getattr(self.node, 'mapper', None)
        if existing is not None:
            self.mapper = existing
            return

        # num_cameras is the per-integrate batch size. Each camera strategy
        # integrates ONE frame per ROS callback, so the Mapper is built for a
        # single-camera frame; multiple cameras simply call integrate() in turn
        # and the shared TSDF fuses them. (All cameras must share image_height/
        # image_width since the projective buffer is sized once here.)
        self.mapper = Mapper(MapperCfg(
            extent_meters_xyz=tuple(self._mapper_extent_xyz),
            # Pin the ESDF extent to the TSDF extent so the produced ESDF grid
            # dims match the pre-allocated voxel collision_cache exactly.
            extent_esdf_meters_xyz=tuple(self._mapper_extent_xyz),
            voxel_size=self._mapper_voxel_size,
            esdf_voxel_size=self._esdf_voxel_size,
            grid_center=torch.tensor(self._mapper_grid_center, dtype=torch.float32),
            image_height=int(self._mapper_image_height),
            image_width=int(self._mapper_image_width),
            depth_minimum_distance=self._mapper_depth_min,
            depth_maximum_distance=self._mapper_depth_max,
            num_cameras=1,
        ))
        self.node.mapper = self.mapper
        self.node.get_logger().info(
            f"Mapper configured: extent={self._mapper_extent_xyz}m, "
            f"tsdf={self._mapper_voxel_size}m, esdf={self._esdf_voxel_size}m, "
            f"image={self._mapper_image_width}x{self._mapper_image_height} "
            f"({int(num_cameras)} camera(s) feeding the shared TSDF)"
        )

    def refresh_esdf(self) -> bool:
        """Recompute the ESDF from the Mapper and stage it into the Scene.

        Returns True if an ESDF voxel grid was produced (the caller should then
        push the scene to the solvers via update_world). No-op without a Mapper.
        """
        if self.mapper is None:
            return False
        try:
            vg = self.mapper.compute_esdf()
        except Exception as e:
            self.node.get_logger().warn(
                f"compute_esdf failed: {e}", throttle_duration_sec=5.0
            )
            return False
        if vg is None:
            return False
        # Single perception voxel grid in the Scene (replaces any previous one).
        self.scene.voxel = [vg]
        self._esdf_voxel_name = vg.name
        return True

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
        self._notify_world_changed()
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
                    self._notify_world_changed()
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
        self._notify_world_changed()
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

        # Bounds derive from the Mapper grid (single source of truth), not a
        # hardcoded extent: grid_center ± extent/2.
        center = np.array(self._mapper_grid_center, dtype=np.float32)
        half = np.array(self._mapper_extent_xyz, dtype=np.float32) / 2.0
        grid_min = center - half
        grid_max = center + half
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
        # v1 SetCollisionCache carries an obb/mesh/blox triple; v2 expects a
        # dict {"cuboid": N, "mesh": N, "voxel": N}. We map obb→cuboid,
        # mesh→mesh, blox→voxel. Negative values are ignored (keep current).
        if request.obb >= 0:
            self.collision_cache["cuboid"] = int(request.obb)
        if request.mesh >= 0:
            self.collision_cache["mesh"] = int(request.mesh)
        if request.blox >= 0:
            self.collision_cache["voxel"] = {
                "layers": int(request.blox),
                "dims": list(self._mapper_extent_xyz),
                "voxel_size": self._esdf_voxel_size,
            }
        elif request.blox == -1:
            self.collision_cache["voxel"] = None

        node.get_logger().info(f'collision_cache set to {self.collision_cache}')
        # The cache size is fixed at solver creation, so a world update is not
        # enough — rebuild the solvers now (observer registered by ConfigWrapper).
        self._notify_cache_changed()

        response.success = True
        response.message = 'Collision cache updated'
        response.obb_cache = self.collision_cache["cuboid"]
        response.mesh_cache = self.collision_cache["mesh"]
        voxel = self.collision_cache["voxel"]
        response.blox_cache = voxel["layers"] if isinstance(voxel, dict) else -1
        return response

    # ---- Getters ----

    def get_scene(self) -> Scene:
        """Return the authoritative Scene (single source of truth)."""
        return self.scene
