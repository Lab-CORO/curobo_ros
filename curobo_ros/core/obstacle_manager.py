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
        # Time (s) for an unobserved voxel to lose half its TSDF weight. Expressed
        # in seconds rather than as a raw per-call factor so it stays meaningful
        # when cameras are added/removed or their frame rate changes — see
        # _resolve_time_decay. <= 0 disables the decay entirely.
        self._decay_half_life_s = self._declare_param('decay_half_life_s', 0.7)
        self._warn_removed_decay_factor()

    def _warn_removed_decay_factor(self):
        """Flag a leftover `decay_factor` override from a pre-normalisation launch.

        rclpy silently keeps overrides for parameters that are never declared, so
        a stale `decay_factor: 0.9` in a launch file would be dropped without a
        word while the operator believes it is still tuning the decay.
        """
        try:
            overrides = self.node.get_node_options().parameter_overrides
        except Exception:
            return
        if any(p.name == 'decay_factor' for p in overrides):
            self.node.get_logger().warn(
                "Parameter 'decay_factor' is set but no longer exists: the TSDF "
                "decay is now derived from 'decay_half_life_s' (seconds) and the "
                "cameras' frame_rate_hz. The provided value is IGNORED - remove "
                "it and set 'decay_half_life_s' instead."
            )

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

    def _resolve_time_decay(self, total_frame_rate_hz: float) -> float:
        """Convert `decay_half_life_s` into cuRobo's per-integrate decay factor.

        cuRobo multiplies the weight of EVERY TSDF voxel by `decay_factor` once
        per `integrate()` call, and each camera integrates its own frames into the
        shared TSDF. The real decay rate is therefore R = sum of the camera frame
        rates, not one decay per "round" — so a raw factor silently changes
        meaning whenever a camera is added or a frame rate is retuned.

        Anchoring on a half-life removes that coupling:

            d = 0.5 ** (1 / (R * half_life))   =>   d ** (R * half_life) = 0.5

        Args:
            total_frame_rate_hz: Combined integrate() rate R, in Hz.

        Returns:
            A decay factor in (0, 1]; 1.0 means "no decay".
        """
        half_life = float(self._decay_half_life_s)
        if half_life <= 0.0:
            self.node.get_logger().info(
                "decay_half_life_s <= 0: TSDF decay disabled (voxels never fade)."
            )
            return 1.0
        if total_frame_rate_hz <= 0.0:
            self.node.get_logger().warn(
                "No camera frame rate available: cannot convert "
                f"decay_half_life_s={half_life}s into a per-integrate factor. "
                "Falling back to no decay (the map will accumulate). Declare "
                "'frame_rate_hz' on each camera in the cameras config."
            )
            return 1.0

        # MapperCfg rejects anything outside (0, 1] (mapper_cfg.py validation).
        decay = 0.5 ** (1.0 / (total_frame_rate_hz * half_life))
        decay = min(max(decay, 1e-6), 1.0)
        if decay > 0.9999:
            self.node.get_logger().warn(
                f"Resolved TSDF decay factor {decay:.6f} is numerically inert "
                f"(half_life={half_life}s at {total_frame_rate_hz} Hz): occupancy "
                "will accumulate without bound. Shorten decay_half_life_s."
            )
        return decay

    def setup_perception(self, num_cameras: int = 1, total_frame_rate_hz: float = 0.0):
        """Create the v2 Mapper and expose it as `node.mapper`.

        Idempotent: if a Mapper already exists on the node (e.g. created by
        another ConfigWrapper sharing this node), adopt it instead of building
        a second one. No-op when `use_mapper` is False.

        Args:
            num_cameras: number of cameras feeding the shared Mapper (sizes the
                projective scratch buffer).
            total_frame_rate_hz: combined integrate() rate of those cameras, used
                to normalise the decay (see _resolve_time_decay).
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
        time_decay = self._resolve_time_decay(total_frame_rate_hz)
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
            # On ne le règle PAS à la main : il est dérivé de `decay_half_life_s`
            # (secondes) et du débit combiné des caméras, parce que cuRobo applique
            # la décroissance UNE FOIS PAR APPEL integrate() — ajouter une caméra
            # ou changer un fps modifierait sinon l'horizon d'oubli en silence.
            decay_factor=time_decay,
            num_cameras=1,
        ))
        self.node.mapper = self.mapper
        self.node.get_logger().info(
            f"Mapper configured: extent={self._mapper_extent_xyz}m, "
            f"tsdf={self._mapper_voxel_size}m, esdf={self._esdf_voxel_size}m, "
            f"image={self._mapper_image_width}x{self._mapper_image_height} "
            f"({int(num_cameras)} camera(s) feeding the shared TSDF), "
            f"decay: half_life={self._decay_half_life_s}s @ R={total_frame_rate_hz}Hz "
            f"-> time_decay={time_decay:.4f}"
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
        # Sanity-check the perception ESDF before it reaches the collision
        # kernels. A corrupt grid (NaN/Inf values, or more voxels than the grid
        # the collision cache was sized for) makes CuRobo's collision kernel read
        # out of bounds → GPU illegal access → host SIGSEGV (exit -11). Reject it
        # here rather than crash the whole node mid-plan.
        if not self._inspect_esdf(vg):
            return False
        # Single perception voxel grid in the Scene (replaces any previous one).
        self.scene.voxel = [vg]
        self._esdf_voxel_name = vg.name
        return True

    def _inspect_esdf(self, vg) -> bool:
        """Validate a perception ESDF voxel grid before it is pushed to the
        collision solvers.

        Returns False (reject the grid) only on *definite* corruption — non-finite
        values, or a voxel count that exceeds the configured grid (the "array
        overflow" that segfaults the collision kernel). A merely cluttered scene
        (high occupancy) is logged but still allowed to plan. The inspection is
        wrapped so it can never itself break planning.
        """
        try:
            ft = getattr(vg, 'feature_tensor', None)
            if ft is None or ft.numel() == 0:
                return True  # not voxelised yet — nothing to check

            n = int(ft.numel())
            n_bad = int((~torch.isfinite(ft)).sum().item())

            if n_bad > 0:
                self.node.get_logger().error(
                    f"Perception ESDF has {n_bad}/{n} non-finite (NaN/Inf) values "
                    f"- rejecting to avoid a collision-kernel segfault "
                    f"(dims={getattr(vg, 'dims', '?')}).",
                    throttle_duration_sec=2.0)
                return False

            # Expected voxel count from the configured mapper extent / resolution.
            vsize = float(getattr(vg, 'voxel_size', 0.0)) or self._esdf_voxel_size
            expected = int(np.prod(np.ceil(
                np.array(self._mapper_extent_xyz, dtype=np.float64) / vsize)))
            if expected and n > int(expected * 1.01) + 1:
                # More voxels than the grid the collision cache was allocated for:
                # the exact "array overflow" that indexes out of bounds in the
                # kernel. Reject — letting it through would segfault the node.
                self.node.get_logger().error(
                    f"Perception ESDF voxel count {n} exceeds expected {expected} "
                    f"(voxel_size={vsize}, extent={self._mapper_extent_xyz}) - "
                    f"rejecting to avoid a collision-kernel overflow.",
                    throttle_duration_sec=2.0)
                return False

            # Occupancy diagnostic — informational only. A value near 1.0 is the
            # "entire area is in collision" state seen right before the crash;
            # note: unobserved cells can bias this, so it is a heuristic, not a
            # rejection criterion.
            occ = float((ft <= 0).float().mean().item())
            if occ > 0.98:
                self.node.get_logger().warn(
                    f"Perception ESDF looks saturated: ~{occ*100:.1f}% of "
                    f"{n} voxels have SDF<=0. Planning will likely report "
                    f"'Start or End state in collision' - check the mapper/TSDF "
                    f"input (stale depth, bad extrinsics, empty cloud).",
                    throttle_duration_sec=2.0)
            else:
                self.node.get_logger().debug(
                    f"Perception ESDF pushed: {n} voxels, ~{occ*100:.1f}% occupied "
                    f"(sdf min={float(ft.min()):.3f} max={float(ft.max()):.3f})",
                    throttle_duration_sec=2.0)
            return True
        except Exception as e:
            # Diagnostics must never break planning — let the grid through.
            self.node.get_logger().warn(
                f"ESDF inspection skipped (error: {e})", throttle_duration_sec=5.0)
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
        # Per-source occupancy, logged together at the end. Split out because a
        # 100%-occupied grid is otherwise indistinguishable between "perception
        # says everything is solid" and "primitive rasterization floods it".
        n_occ = n_prim = 0

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
            except Exception as e:
                node.get_logger().warn(f'Mapper perception query failed: {e}')

        # ---- Analytic primitives (AddObject obstacles) — GPU path ----
        # Always rasterized: cuboids/spheres/etc. added via add_object must show
        # up in the voxel grid (just like the camera perception). The collision
        # kernels are Warp-based and disk-cached (/root/.cache/warp), so the
        # first call does not recompile (~0.02 s measured in a cold process). The
        # node additionally pre-warms this path at startup via
        # prewarm_voxel_rasterization() (covers the rare empty-Warp-cache case).
        n_obs = sum(len(getattr(self.scene, b) or [])
                    for b in ('cuboid', 'sphere', 'capsule', 'cylinder', 'mesh'))
        if n_obs > 0:
            n_prim = self._rasterize_primitives_gpu(node, voxel_grid, grid_min, voxel_size, size)

        # Attributes each occupied cell to its source: n_occ is the Mapper's
        # observed-and-inside voxels, n_prim is what _rasterize_primitives_gpu
        # flags against analytic primitives only (see primitives_only_scene()).
        total = int(size[0]) * int(size[1]) * int(size[2])
        n_final = int(voxel_grid.sum())
        node.get_logger().debug(
            f'Voxel grid: mapper={n_occ} primitives={n_prim} '
            f'final={n_final}/{total} ({100.0 * n_final / total:.1f}% occupied)',
            throttle_duration_sec=2.0)

        return voxel_grid, grid_min, size

    def get_voxel_grid(self, node, request: GetVoxelGrid, response):
        """Return occupied voxels for the full scene (Mapper ESDF + analytic primitives).

        Analytic primitives are rasterized via a pre-allocated GPU SceneCollision
        query (batch of point-spheres with r=0), using the same collision checker
        as the planner (exact consistency). No CPU fallback: on a GPU
        rasterization failure, this returns a size 0x0x0 grid rather than
        raising — the service still responds (an uncaught exception here would
        leave the caller blocked until its own client-side timeout, since a
        service callback that raises never sends a reply).
        """
        voxel_size = self._resolve_voxel_size(node)
        try:
            voxel_grid, grid_min, size = self._compute_dense_voxel_grid(node, voxel_size)
        except Exception as e:
            node.get_logger().error(f'get_voxel_grid: computation failed ({e})')
            response.voxel_grid.resolutions = rnp.msgify(
                Vector3, np.array([voxel_size, voxel_size, voxel_size])
            )
            response.voxel_grid.size_x = 0
            response.voxel_grid.size_y = 0
            response.voxel_grid.size_z = 0
            response.voxel_grid.data = []
            return response

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

        On a GPU rasterization failure, this cycle is skipped entirely — no
        partial grid (missing the analytic primitives) is published. A silent
        topic is an unambiguous signal of a stalled publisher; a grid quietly
        missing obstacles is not distinguishable from "no obstacles there".
        """
        voxel_size = self._resolve_voxel_size(node)
        try:
            voxel_grid, grid_min, size = self._compute_dense_voxel_grid(node, voxel_size)
        except Exception as e:
            node.get_logger().warn(
                f'Voxel grid computation failed ({e}) - skipping this publish '
                f'cycle rather than publish a grid without analytic primitives',
                throttle_duration_sec=5.0)
            return

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

    def prewarm_voxel_rasterization(self, node) -> None:
        """Pre-warm the GPU primitive-rasterization path.

        Builds the voxel `SceneCollision` and runs a dummy query so that any
        Warp kernel compilation/loading happens AT STARTUP (called from the
        node's ``__init__``, before the executor serves any service) rather than
        on the first ``get_voxel_grid`` call.

        On this machine the Warp kernels are already disk-cached (~0.02 s), but
        this pre-warm also covers an empty Warp cache (very first boot). It works
        even with an empty scene (the kernel is still exercised). Silent no-op on
        failure — it must never block node startup.
        """
        try:
            import time as _time
            t0 = _time.perf_counter()
            voxel_size = self._resolve_voxel_size(node)
            center = np.array(self._mapper_grid_center, dtype=np.float32)
            half = np.array(self._mapper_extent_xyz, dtype=np.float32) / 2.0
            grid_min = center - half
            grid_max = center + half
            size = np.ceil((grid_max - grid_min) / voxel_size).astype(np.int32)
            voxel_grid = np.zeros((size[0], size[1], size[2]), dtype=np.uint32)
            self._rasterize_primitives_gpu(node, voxel_grid, grid_min, voxel_size, size)
            node.get_logger().info(
                f"Voxel primitive rasterization pre-warmed in "
                f"{_time.perf_counter() - t0:.2f}s")
        except Exception as e:
            node.get_logger().warn(f"Voxel rasterization pre-warm skipped: {e}")

    def _rasterize_primitives_gpu(self, node, voxel_grid, grid_min, voxel_size, size) -> int:
        """Rasterize analytic primitives using a pre-allocated GPU SceneCollision query.

        Builds a flat grid of point-spheres (r=0) covering the whole extent, sends
        them as a single batch to ``get_sphere_distance_raw``, and marks cells where
        distance > 0 as occupied (cuRobo convention: positive = in collision).

        The SceneCollision and GPU buffers are pre-allocated on first call and
        reused across calls. They are only rebuilt when the grid parameters change
        (voxel_size or extent). The scene is synced via ``load_collision_model``
        on every call to reflect the current set of obstacles.

        The scene handed to this query is ANALYTIC PRIMITIVES ONLY — see
        primitives_only_scene() for why the perception voxel layer must stay out.
        """
        device = f'cuda:{torch.cuda.current_device()}' if torch.cuda.is_available() else 'cpu'
        dtype = torch.float32
        primitives_scene = self.primitives_only_scene()

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
            cfg = SceneCollisionCfg(
                device_cfg=dev_cfg,
                scene_model=primitives_scene,
            )
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
        self._voxel_sc.load_collision_model(primitives_scene)

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

    def primitives_only_scene(self) -> Scene:
        """The current Scene minus the perception ESDF voxel layer.

        Scene.objects is assembled from every bucket *including* `voxel`, so
        handing self.scene to a SceneCollision query also queries the nvblox ESDF
        layer. That is wrong for the voxel-grid rasterization for two reasons:

        1. Double-counting. _compute_dense_voxel_grid already fills perception
           from the Mapper via extract_occupied_voxels(), which reads the TSDF and
           therefore reports only *observed* cells.
        2. Correctness. Querying the dense ESDF re-introduces the unobserved-cell
           ambiguity that the Mapper branch exists to avoid: unobserved cells are
           not "far from an obstacle", so they read as in-collision. Benign while
           the ESDF is barely populated, but once MPC starts refreshing perception
           on its servo loop the query saturates — measured 2026-07-25, primitives
           went 124620 -> 2097152 (100% of the grid) at the first MPC step while
           the Mapper count stayed healthy at ~13.5k.

        It is also what a solver must be *constructed* with. SceneData.from_scene_cfg
        ignores the pre-allocated voxel cache whenever the scene already carries a
        voxel layer, and VoxelData.from_scene_cfg then takes the zero-copy branch
        (create_from_voxel_grids: `features = feature_tensor.view(...)`), aliasing
        the solver's collision buffer onto our live ESDF tensor. Every later
        update_world does clear() -> re-add, and clear() writes -max_esdf_distance
        into that aliased buffer -- i.e. into our source tensor -- so the re-add
        copies -100 ("solid everywhere") onto itself. Passing this primitives-only
        Scene instead makes the constructor honour collision_cache["voxel"] and
        allocate its own buffer, after which update_world copies normally. See the
        2026-07-25 debug: MPC was built after perception was live and read
        -100 everywhere, while MotionGen -- built at startup, before any ESDF
        existed -- got a real cache and stayed healthy.

        Cheap: the lists are shared by reference, only the container is new.
        """
        return Scene(
            cuboid=self.scene.cuboid,
            sphere=self.scene.sphere,
            capsule=self.scene.capsule,
            cylinder=self.scene.cylinder,
            mesh=self.scene.mesh,
        )

    def set_collision_cache(
        self,
        node,
        request: SetCollisionCache.Request,
        response: SetCollisionCache.Response,
    ) -> SetCollisionCache.Response:
        """
        v2: a single `collision_cache` dict replaces the v1 obb/mesh/blox
        triple. Contract (see SetCollisionCache.srv): -1 = leave this field
        unchanged, 0 = disable/empty this cache, >0 = new size.
        """
        # v1 SetCollisionCache carries an obb/mesh/blox triple; v2 expects a
        # dict {"cuboid": N, "mesh": N, "voxel": N}. We map obb→cuboid,
        # mesh→mesh, blox→voxel.
        if request.obb >= 0:
            self.collision_cache["cuboid"] = int(request.obb)
        if request.mesh >= 0:
            self.collision_cache["mesh"] = int(request.mesh)
        if request.blox == 0:
            # Explicit disable: no voxel storage pre-allocated in the solvers
            # -> camera-based (ESDF) collision avoidance will not work.
            self.collision_cache["voxel"] = None
        elif request.blox > 0:
            self.collision_cache["voxel"] = {
                "layers": int(request.blox),
                "dims": list(self._mapper_extent_xyz),
                "voxel_size": self._esdf_voxel_size,
            }
        # request.blox < 0 -> leave voxel cache unchanged (contract: -1 = no change)

        node.get_logger().info(f'collision_cache set to {self.collision_cache}')
        # The cache size is fixed at solver creation, so a world update is not
        # enough — rebuild the solvers now (observer registered by ConfigWrapper).
        self._notify_cache_changed()

        response.success = True
        response.message = 'Collision cache updated'
        response.obb_cache = self.collision_cache["cuboid"]
        response.mesh_cache = self.collision_cache["mesh"]
        voxel = self.collision_cache["voxel"]
        response.blox_cache = voxel["layers"] if isinstance(voxel, dict) else 0
        return response

    # ---- Getters ----

    def get_scene(self) -> Scene:
        """Return the authoritative Scene (single source of truth)."""
        return self.scene
