import os
import numpy as np
import torch
import ros2_numpy as rnp

from std_srvs.srv import Trigger
from curobo_msgs.srv import AddObject, RemoveObject, GetVoxelGrid, SetCollisionCache
from curobo_msgs.msg import SparseVoxelGrid
from geometry_msgs.msg import Vector3

from curobo.scene import Scene, Cuboid, Capsule, Cylinder, Sphere, Mesh
from curobo.perception import Mapper, MapperCfg
from curobo._src.geom.collision.collision_scene import SceneCollisionCfg, SceneCollision
from curobo._src.geom.collision.buffer_collision import CollisionBuffer
from curobo.types import DeviceCfg


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

        # GPU voxel query buffers (pre-allocated, rebuilt only when grid changes).
        # Avoids per-call CPU trimesh; uses the scene's own collision checker.
        self._voxel_sc = None           # SceneCollision
        self._voxel_spheres = None      # Tensor [1,1,N,4] on GPU
        self._voxel_buf = None          # CollisionBuffer
        self._voxel_weight = None       # Tensor [1]
        self._voxel_act = None          # Tensor [1]
        self._voxel_grid_key = None     # (grid_min tuple, voxel_size, size tuple)

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
        # Whether to rasterize analytic primitives into the voxel grid via the
        # GPU SceneCollision path. Disabled by default because the first call
        # triggers NVRTC kernel compilation (~90 s warmup). Enable it once the
        # system is fully up with:
        #   ros2 param set /<node> enable_primitives_rasterization true
        # Read dynamically so it can be toggled at runtime without restarting.
        self._declare_param('enable_primitives_rasterization', False)

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
        # Invalidate the grid key so the next get_voxel_grid call reloads the
        # scene into the GPU SceneCollision (the buffers themselves are reused).
        self._voxel_grid_key = None
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
            # CuRobo v0.8 MapperCfg infers image dims from the depth frame at
            # integration time — it no longer takes image_height/image_width.
            depth_minimum_distance=self._mapper_depth_min,
            depth_maximum_distance=self._mapper_depth_max,
            # decay_factor défaut=1.0 = AUCUNE décroissance → l'occupancy accumule
            # sans fin et le nombre de voxels occupés croît de façon monotone même
            # sur une scène statique (bruit de bord franchissant peu à peu le seuil).
            # <1.0 fait décroître les vieilles observations → carte stationnaire.
            decay_factor=0.95,
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

    def _resolve_voxel_size(self, node) -> float:
        """Resolve the active voxel resolution from the `voxel_size` param."""
        voxel_size = node.get_parameter('voxel_size').get_parameter_value().double_value
        if voxel_size <= 0.0:
            voxel_size = self._esdf_voxel_size
        return voxel_size

    def _compute_dense_voxel_grid(self, node, voxel_size):
        """Build the dense occupancy grid for the full scene.

        Combines the Mapper ESDF (camera perception) and analytic primitives
        (AddObject obstacles, rasterized on GPU via SceneCollision). Shared by
        the get_voxel_grid service and the sparse voxel topic publisher.

        Logs at debug level so the periodic topic publisher does not flood the
        console at its publish rate.

        Returns:
            (voxel_grid, grid_min, size):
                voxel_grid : np.ndarray[uint32] of shape (size_x, size_y, size_z),
                             1 = occupied, 0 = free.
                grid_min   : np.ndarray[float32] bbox origin (metres, base frame).
                size       : np.ndarray[int32] grid dimensions in voxels.
        """
        center = np.array(self._mapper_grid_center, dtype=np.float32)
        half = np.array(self._mapper_extent_xyz, dtype=np.float32) / 2.0
        grid_min = center - half
        grid_max = center + half
        size = np.ceil((grid_max - grid_min) / voxel_size).astype(np.int32)

        voxel_grid = np.zeros((size[0], size[1], size[2]), dtype=np.uint32)

        # ---- Mapper perception (camera-based) ----
        # extract_occupied_voxels() returns only the *observed* occupied voxel
        # centres (from the TSDF), so it sidesteps the dense-ESDF ambiguity where
        # unobserved cells are zero-initialised and get mislabelled.
        #
        # CuRobo v2 (v0.8) : la méthode vit sur l'intégrateur ESDF
        # (mapper.integrator -> BlockSparseESDFIntegrator), PAS sur le Mapper, et
        # renvoie un tuple (centers, colors) — et non un objet avec .centers.
        #   centers : (N, 3) float32, positions monde des voxels avec SDF <= 0.
        mapper = getattr(node, 'mapper', None)
        if mapper is not None:
            try:
                centers, _colors = mapper.integrator.extract_occupied_voxels(surface_only=False)
                n_occ = 0 if centers is None else int(centers.shape[0])
                if n_occ:
                    world_xyz = centers.detach().cpu().numpy()  # [N, 3] world
                    idx = np.floor((world_xyz - grid_min) / voxel_size).astype(np.int64)
                    valid = np.all((idx >= 0) & (idx < size), axis=1)
                    idx = idx[valid]
                    voxel_grid[idx[:, 0], idx[:, 1], idx[:, 2]] = 1
                node.get_logger().debug(
                    f'Filled voxel grid from Mapper perception ({n_occ} occupied cells)')
            except Exception as e:
                node.get_logger().warn(f'Mapper perception query failed: {e}')

        # ---- Analytic primitives (AddObject obstacles) — GPU path ----
        # Gated by `enable_primitives_rasterization` because the first call
        # allocates SceneCollision and triggers NVRTC kernel compilation (~90 s).
        # Read the param fresh each call so it can be toggled at runtime:
        #   ros2 param set /<node> enable_primitives_rasterization true
        enable_rasterization = node.get_parameter('enable_primitives_rasterization').value
        n_obs = sum(len(getattr(self.scene, b) or [])
                    for b in ('cuboid', 'sphere', 'capsule', 'cylinder', 'mesh'))
        if n_obs > 0 and enable_rasterization:
            try:
                n_prim = self._rasterize_primitives_gpu(node, voxel_grid, grid_min, voxel_size, size)
                node.get_logger().debug(
                    f'Filled voxel grid from analytic primitives ({n_prim} occupied cells)')
            except Exception as e:
                node.get_logger().warn(f'GPU primitive voxelization failed ({e}), falling back to CPU')
                try:
                    n_prim = self._rasterize_primitives_cpu(node, voxel_grid, grid_min, voxel_size, size)
                    if n_prim:
                        node.get_logger().debug(
                            f'Filled voxel grid from analytic primitives (CPU, {n_prim} cells)')
                except Exception as e2:
                    node.get_logger().warn(f'CPU primitive voxelization also failed: {e2}')
        elif n_obs > 0:
            node.get_logger().debug(
                f'Skipping primitives rasterization for {n_obs} obstacle(s) '
                '(enable_primitives_rasterization=false)')

        return voxel_grid, grid_min, size

    def get_voxel_grid(self, node, request: GetVoxelGrid, response):
        """Return occupied voxels for the full scene (Mapper ESDF + analytic primitives).

        Analytic primitives are rasterized via a pre-allocated GPU SceneCollision
        query (batch of point-spheres with r=0) — ~29× faster than CPU trimesh
        and uses the same collision checker as the planner (exact consistency).
        Falls back to CPU trimesh if the GPU path is unavailable.
        """
        voxel_size = self._resolve_voxel_size(node)
        voxel_grid, grid_min, size = self._compute_dense_voxel_grid(node, voxel_size)

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

    def publish_sparse_voxel_grid(self, node, publisher):
        """Publish the current scene occupancy as a sparse SparseVoxelGrid message.

        Only the C-order linear indices of occupied voxels are sent
        (linear = x * size_y * size_z + y * size_z + z), which is typically
        1–2 orders of magnitude smaller on the wire than the dense grid.
        Driven by the periodic timer in RosServiceManager.
        """
        voxel_size = self._resolve_voxel_size(node)
        voxel_grid, grid_min, size = self._compute_dense_voxel_grid(node, voxel_size)

        # C-order flatten matches the message convention exactly.
        occupied = np.flatnonzero(voxel_grid).astype(np.int32)

        msg = SparseVoxelGrid()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = self.config_manager.get_base_link()
        msg.origin.x = float(grid_min[0])
        msg.origin.y = float(grid_min[1])
        msg.origin.z = float(grid_min[2])
        msg.resolution = float(voxel_size)
        msg.size_x = int(size[0])
        msg.size_y = int(size[1])
        msg.size_z = int(size[2])
        msg.occupied_indices = occupied.tolist()
        publisher.publish(msg)

    def _rasterize_primitives_gpu(self, node, voxel_grid, grid_min, voxel_size, size) -> int:
        """Rasterize analytic primitives using a pre-allocated GPU SceneCollision query.

        Builds a flat grid of point-spheres (r=0) covering the whole extent, sends
        them as a single batch to ``get_sphere_distance_raw``, and marks cells where
        distance > 0 as occupied (cuRobo convention: positive = in collision).

        The SceneCollision and GPU buffers are pre-allocated on first call and
        reused across calls. They are only rebuilt when the grid parameters change
        (voxel_size or extent). The scene is synced via ``load_collision_model``
        on every call to reflect the current set of obstacles.
        """
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        dtype = torch.float32

        # Build grid centers [N, 3] — same layout as get_voxel_grid
        grid_key = (tuple(grid_min.tolist()), float(voxel_size), tuple(size.tolist()))
        if self._voxel_grid_key != grid_key:
            ax = np.arange(size[0]); ay = np.arange(size[1]); az = np.arange(size[2])
            gx, gy, gz = np.meshgrid(ax, ay, az, indexing='ij')
            idx = np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)
            centers = (grid_min + (idx + 0.5) * voxel_size).astype(np.float32)
            n_pts = len(centers)

            dev_cfg = DeviceCfg(device=device)
            # SceneCollision is built fresh — it will be synced with the scene below.
            cfg = SceneCollisionCfg(device_cfg=dev_cfg, scene_model=self.scene)
            self._voxel_sc = SceneCollision.from_config(cfg)

            # [1, 1, N, 4] — batch=1, horizon=1, N point-spheres, radius=0
            self._voxel_spheres = torch.zeros((1, 1, n_pts, 4), device=device, dtype=dtype)
            self._voxel_spheres[0, 0, :, :3] = torch.tensor(centers, dtype=dtype, device=device)

            self._voxel_buf = CollisionBuffer.from_shape(
                torch.Size([1, 1, n_pts]), dev_cfg)
            self._voxel_weight = torch.ones(1, device=device, dtype=dtype)
            self._voxel_act = torch.zeros(1, device=device, dtype=dtype)
            self._voxel_grid_key = grid_key

        # Sync current scene obstacles into the checker (cheap — no GPU realloc).
        self._voxel_sc.load_collision_model(self.scene)

        self._voxel_buf.zero_()
        self._voxel_sc.get_sphere_distance_raw(
            self._voxel_spheres, self._voxel_buf,
            self._voxel_weight, self._voxel_act,
        )
        # distance > 0 means the point-sphere is in collision with an obstacle.
        occ_mask = (self._voxel_buf.distance[0, 0] > 0).cpu().numpy()
        n_pts = self._voxel_spheres.shape[2]
        sx, sy, sz = int(size[0]), int(size[1]), int(size[2])
        occ_flat = occ_mask.reshape(sx, sy, sz)
        voxel_grid[:] |= occ_flat.astype(np.uint32)
        return int(occ_mask.sum())

    def _rasterize_primitives_cpu(self, node, voxel_grid, grid_min, voxel_size, size) -> int:
        """CPU trimesh fallback for analytic primitive rasterization."""
        grid_min = np.asarray(grid_min, dtype=np.float64)
        size = np.asarray(size, dtype=np.int64)
        total = 0
        for bucket in ('cuboid', 'sphere', 'capsule', 'cylinder', 'mesh'):
            for obs in (getattr(self.scene, bucket, None) or []):
                try:
                    mesh = obs.get_trimesh_mesh(transform_with_pose=True)
                    lo = np.asarray(mesh.bounds[0], dtype=np.float64)
                    hi = np.asarray(mesh.bounds[1], dtype=np.float64)
                    i0 = np.maximum(np.floor((lo - grid_min) / voxel_size).astype(np.int64), 0)
                    i1 = np.minimum(np.ceil((hi - grid_min) / voxel_size).astype(np.int64), size)
                    if np.any(i1 <= i0):
                        continue
                    ax = np.arange(i0[0], i1[0])
                    ay = np.arange(i0[1], i1[1])
                    az = np.arange(i0[2], i1[2])
                    gx, gy, gz = np.meshgrid(ax, ay, az, indexing='ij')
                    idx = np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)
                    centers = grid_min + (idx + 0.5) * voxel_size
                    inside = mesh.contains(centers)
                    sel = idx[inside]
                    if len(sel):
                        voxel_grid[sel[:, 0], sel[:, 1], sel[:, 2]] = 1
                    total += int(inside.sum())
                except Exception as e:
                    node.get_logger().warn(
                        f"voxelize (CPU): skipped '{getattr(obs, 'name', '?')}' ({e})")
        return total

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
