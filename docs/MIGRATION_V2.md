# Migration to cuRobo v2 (v0.8.0)

Reference for developers of the `curobo_ros` wrapper itself: how the code base maps onto **cuRobo v2 (v0.8.0)**, a major rewrite of the upstream API, and which v1 concepts no longer exist. Regular users do not need this page — the ROS interfaces were unaffected.

The migration is **complete**: the package contains no `MotionGen`, `WorldConfig`, `TensorDeviceType`, `IKSolver`, or nvblox code. `node.motion_gen` survives only as an alias to `node.motion_planner` for old call sites.

## What v2 brought

- **Mapper** — native block-sparse TSDF + GPU ESDF (unified depth → SDF pipeline), replacing the `Lab-CORO/nvblox` + `nvblox_torch` forks
- **MotionPlanner** — one API for single/batch/goalset/multi-env planning
- **Tool-frame types** — `Pose` / `ToolPose` / `GoalToolPose`
- **Dynamics-aware B-spline trajopt**
- Composition-based architecture, easier to extend

This let the wrapper delete: the `Lab-CORO/curobo` fork (branch `lab-coro`), the nvblox forks, the homegrown `MeshBloxilization` voxelizer, the mesh+cuboid dual storage in `ObstacleManager`, the triple `obb`/`mesh`/`blox` cache (now a single v2 `collision_cache` dict — the `SetCollisionCache` service keeps its three request fields and maps them onto it), and `MultiPointPlanner`'s manual per-waypoint loop.

## Prerequisites

| Component | v1 | v2 |
|---|---|---|
| Python | ≥ 3.8 | **≥ 3.10** |
| Torch | 2.0+ | ≥ 2.5 (cu12) / ≥ 2.9 (cu13) |
| CUDA | 11+ | **≥ 12** |
| ROS 2 | Humble | Humble or Jazzy |

## Import mapping

| v1 | v2 |
|---|---|
| `curobo.wrap.reacher.motion_gen` → `MotionGen`, `MotionGenConfig`, `MotionGenPlanConfig`, `MotionGenResult` | `curobo.motion_planner` → `MotionPlanner`, `MotionPlannerCfg` |
| `curobo.wrap.reacher.ik_solver` → `IKSolver`, `IKSolverConfig` | `curobo.inverse_kinematics` → `InverseKinematics`, `InverseKinematicsCfg`, `InverseKinematicsResult` |
| `curobo.wrap.reacher.mpc` → `MpcSolver`, `MpcSolverConfig` | `curobo.model_predictive_control` → `ModelPredictiveControl`, `ModelPredictiveControlCfg`, `ModelPredictiveControlResult` |
| `curobo.cuda_robot_model.cuda_robot_model` → `CudaRobotModel` | `curobo.kinematics` → `Kinematics`, `KinematicsCfg` |
| `curobo.types.base` → `TensorDeviceType` | `curobo.types` → `DeviceCfg` |
| `curobo.types.robot` → `RobotConfig`, `JointState` | `curobo.types` → `JointState`; robot config via the `*.Cfg.create(robot=…)` factories |
| `curobo.types.math` → `Pose` | `curobo.types` → `Pose` (plus `ToolPose`, `GoalToolPose`) |
| `curobo.geom.types` → `WorldConfig`, `Cuboid`, `Capsule`, `Cylinder`, `Sphere`, `Mesh` | `curobo.scene` → `Scene`, `SceneData`, `SceneCfg`, same primitives, `VoxelGrid` |
| `curobo.geom.sdf.world` → `CollisionCheckerType`, `CollisionQueryBuffer` | **Removed** — folded into `MotionPlannerCfg.create(scene_model=…, collision_cache=…, self_collision_check=…)` |
| `curobo.rollout.cost.pose_cost` → `PoseCostMetric` | **Removed from public API** — use `position_tolerance` / `orientation_tolerance` and `GoalToolPose` constraints |

Perception (not publicly re-exported in v0.8.0 — imported from `_src`):

```python
from curobo._src.perception.mapper.mapper import Mapper
from curobo._src.perception.mapper.mapper_cfg import MapperCfg
```

## Behavioral changes

**`WorldConfig` → `Scene`.** `Scene` is a flat dataclass; world YAMLs load through `Scene.from_dict(...)`. One wrapper-specific trap: never construct a solver from a `Scene` that already carries the live perception voxel layer — use `ObstacleManager.primitives_only_scene()` (the collision cache allocates the voxel storage; `update_world` fills it by copy).

**`plan_single(Pose)` → `plan_pose(GoalToolPose)`.**

```python
goal = GoalToolPose(
    tool_frames=["tcp"],
    position=pos_tensor,      # [B, H, L, G, 3]
    quaternion=quat_tensor,   # [B, H, L, G, 4]
)
result = planner.plan_pose(goal, start_state, max_attempts=5)
```

Waypoint sequences use the `L` dimension of the batched tensor; goalsets use `G`.

**MPC.** `ModelPredictiveControl(ModelPredictiveControlCfg.create(robot=…, scene_model=…, optimization_dt=…, num_control_points=…))`, then `optimize_next_action(current_state)` in the control loop. See [MPC Implementation](concepts/mpc-implementation.md) for how `curobo_ros` wraps it (`ReactiveController` / `MPCController`).

**Perception.** Depth frames are pushed: `mapper.integrate(CameraObservation(depth, intrinsics, pose))`; solvers read the resulting ESDF through the shared `Scene`. No nvblox, no `MeshBloxilization` — a `Mesh` goes straight into the `Scene`.

**CUDA graphs.** The wrapper enables `curobo.runtime.cuda_graph_reset = True` before building any cuRobo object (requires CUDA 12+) and keeps at most one live captured graph across solvers — see [Manager Architecture](concepts/manager-architecture.md).

## Related pages

- [Manager Architecture](concepts/manager-architecture.md) — where these objects live in the wrapper
- [Unified Planner](concepts/unified-planner.md) — the planner layer on top of them
