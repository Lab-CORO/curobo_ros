# Migration cuRobo v1 → v2 (v0.8.0)

Ce document trace la migration du wrapper `curobo_ros` de cuRobo v1
(≤ v0.7.8) vers **cuRobo v2 (v0.8.0, avril 2026)**, un rewrite majeur
de l'API upstream.

## Pourquoi migrer

v2 introduit, en natif :

- **Mapper** — TSDF bloc-sparse + ESDF GPU (pipeline depth → SDF unifié)
- **MotionPlanner** — API unifiée batch/goalset/multi-env
- **Tool-frame** — `Pose` / `ToolPose` / `GoalToolPose`
- **Trajopt B-spline dynamics-aware** (smoothness + couples)
- **`plan_grasp`** — approach + grasp + lift en un appel
- Architecture composition (plus d'héritage), plus facile à étendre

Côté `curobo_ros`, cela permet de supprimer :

- Le fork `Lab-CORO/curobo` (branche `lab-coro`)
- Le fork `Lab-CORO/nvblox` + `nvblox_torch`
- `MeshBloxilization` (voxelisation maison `trimesh` → greedy merge)
- Le dual-storage `mesh` + `cuboid` dans `ObstacleManager`
- La triple cache collision (`obb` / `mesh` / `blox`)
- La boucle `plan_single` manuelle de `MultiPointPlanner`
- Le workaround `std::bad_alloc` PyTorch 2.9 du Dockerfile

## Pré-requis

| Composant | v1 | v2 |
|---|---|---|
| Python | ≥ 3.8 | **≥ 3.10** |
| Torch | 2.0+ | ≥ 2.5 (cu12) / ≥ 2.9 (cu13) |
| CUDA | 11+ | ≥ 12 |
| ROS 2 | Humble | Humble ou Jazzy |

## Mapping des imports

| v1 | v2 |
|---|---|
| `from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig, MotionGenResult` | `from curobo.motion_planner import MotionPlanner, MotionPlannerCfg, GraspPlanResult` |
| `from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig` | `from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg, InverseKinematicsResult` |
| `from curobo.wrap.reacher.mpc import MpcSolver, MpcSolverConfig` | `from curobo.model_predictive_control import ModelPredictiveControl, ModelPredictiveControlCfg, ModelPredictiveControlResult` |
| `from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel` | `from curobo.kinematics import Kinematics, KinematicsCfg` |
| `from curobo.types.base import TensorDeviceType` | `from curobo.types import DeviceCfg` |
| `from curobo.types.robot import RobotConfig, JointState` | `from curobo.types import JointState` ; `RobotCfg` via factory `*.Cfg.create(robot=…)` |
| `from curobo.types.math import Pose` | `from curobo.types import Pose` |
| `from curobo.geom.types import WorldConfig, Cuboid, Capsule, Cylinder, Sphere, Mesh` | `from curobo.scene import Scene, SceneData, SceneCfg, Cuboid, Capsule, Cylinder, Sphere, Mesh, VoxelGrid` |
| `from curobo.geom.sdf.world import CollisionCheckerType, CollisionQueryBuffer` | **supprimé** — intégré à `MotionPlannerCfg.create(scene_model=…, collision_cache=…, self_collision_check=…)` |
| `from curobo.util.trajectory import InterpolateType` | Non exposé publiquement ; alternative : retime natif de `plan_pose` |
| `from curobo.rollout.cost.pose_cost import PoseCostMetric` | **supprimé de l'API publique** ; remplacer par `position_tolerance` / `orientation_tolerance` + contraintes de `GoalToolPose` |
| `from curobo.util_file import load_yaml, get_robot_configs_path, join_path` | Conservé (utilitaires) — à valider |

### Mapper (perception)

Non encore réexporté publiquement en v0.8.0 ; importer depuis `_src` :

```python
from curobo._src.perception.mapper.mapper import Mapper
from curobo._src.perception.mapper.mapper_cfg import MapperCfg
```

## Changements comportementaux

### 1. `WorldConfig` → `Scene`

`Scene` est une dataclass plate. Le YAML world config v1 n'est plus lu
tel quel : il faut passer par `Scene.from_dict(...)` ou construire un
`SceneCfg` via les primitives importées de `curobo.scene`.

### 2. `plan_single(Pose)` → `plan_pose(GoalToolPose)`

```python
# v1
result = motion_gen.plan_single(start_state, goal_pose, MotionGenPlanConfig(...))

# v2
goal = GoalToolPose(
    tool_frames=["tcp"],
    position=pos_tensor,      # [B, H, L, G, 3]
    quaternion=quat_tensor,   # [B, H, L, G, 4]
)
result = planner.plan_pose(goal, start_state, max_attempts=5)
```

### 3. Multi-waypoint natif

Plus de boucle `plan_single` par waypoint. Les waypoints se passent via
le tensor batché `[B, H, L, ...]` où la dimension `L` représente la
séquence.

### 4. MPC

```python
# v2
mpc = ModelPredictiveControl(ModelPredictiveControlCfg.create(
    robot=robot_yml,
    scene_model=scene_cfg,
    horizon=30,
    step_dt=0.03,
))
action_seq = mpc.optimize_action_sequence(current_state)
```

### 5. Perception (pipeline depth → ESDF)

```python
# v2
mapper = Mapper(MapperCfg(...))
obs = CameraObservation(depth=depth_img, intrinsics=K, pose=cam_pose)
mapper.integrate(obs)

voxel_grid = mapper.compute_esdf()
scene.voxel_grid = [voxel_grid]
planner.update_world(scene)
```

Plus besoin de nvblox_torch ni de `MeshBloxilization` — un `Mesh`
s'ajoute directement au `Scene` et `Mapper.update_static_obstacles`
stampe les primitives dans le TSDF.

## Nouvelles features activées

- **Goalset** — tester N candidats de pose en un seul appel via la dim
  `G` de `GoalToolPose`
- **Batch multi-env** — planifier M environnements en parallèle via
  `max_batch_size` + `multi_env=True`
- **Trajopt B-spline** — smoothness + couples exposés via
  `MotionPlannerCfg.create(...)` (paramètres dynamics)
- **Grasp pipeline** — `plan_grasp(approach_axis, lift_offset, …)`
- **`Mapper.refine_pose`** — ICP natif depth vs TSDF

## Zones qui disparaissent du wrapper

| Fichier | Raison |
|---|---|
| `MeshBloxilization` dans `obstacle_manager.py` | `Mesh` géré directement par `Scene` + `Mapper` |
| Dual-storage `world_cfg.mesh` + `world_cfg.cuboid` | `Scene` uniforme |
| Triple cache `obb/mesh/blox` | Un seul `collision_cache` de `MotionPlannerCfg` |
| Boucle waypoint dans `multi_point_planner.py` | Dim `L` de `GoalToolPose` |
| nvblox, nvblox_torch dans Dockerfile | `Mapper` v2 natif |
| Workaround `std::bad_alloc` PyTorch 2.9 | Plus de nvblox → plus de workaround |

## État de la migration

Suivi dans [`RELEASE_CHECKLIST.md`](../RELEASE_CHECKLIST.md) et
l'historique de la branche `curobo_v2`.
