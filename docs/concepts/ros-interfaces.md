# ROS 2 Interfaces Reference

This page is the normative reference for every topic, service, and action exposed by `curobo_ros`. All interface types come from the companion package [`curobo_msgs`](https://github.com/Lab-CORO/curobo_msgs) unless stated otherwise.

The package installs two executables:

| Executable | Node name | Role |
|---|---|---|
| `curobo_trajectory_planner` | `unified_planner` | Motion planning, obstacle management, IK/FK, robot execution |
| `robot_segmentation` | `curobo_depth_map_robot_segmentation` | Removes the robot from depth images before perception |

All planner interfaces are namespaced under the node name: `/unified_planner/<name>`.

## Services — planning

### `/unified_planner/generate_trajectory`

Type: `curobo_msgs/srv/TrajectoryGeneration`. Plans a trajectory with the currently active planner, without executing it.

Request:

| Field | Type | Used by |
|---|---|---|
| `start_pose` | `sensor_msgs/JointState` | All planners. Empty = current robot state |
| `target_pose` | `geometry_msgs/Pose` | Classic planner (single Cartesian goal) |
| `target_poses` | `geometry_msgs/Pose[]` | Multi-point planner (sequence of waypoints) |
| `target_joint_positions` | `float64[]` | Joint-space planner |
| `trajectory_constraints` | `int8[]` | `[theta_x, theta_y, theta_z, x, y, z]` hold flags for the single goal |
| `trajectories_contraints` | `int8[]` | Flattened per-waypoint constraints (note: field name carries a historical typo) |

Response:

| Field | Type | Meaning |
|---|---|---|
| `success` | `bool` | Planning succeeded |
| `message` | `string` | Human-readable status |
| `trajectory` | `sensor_msgs/JointState[]` | Waypoints with position and velocity |
| `dt` | `float64` | Time step between waypoints (seconds) |

The active planner decides which target field it consumes: sending `target_pose` while the joint-space planner is active will not do what you expect. Switch planners first with `set_planner`.

```bash
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
  "{target_pose: {position: {x: 0.5, y: 0.2, z: 0.4}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}"
```

### `/unified_planner/set_planner`

Type: `curobo_msgs/srv/SetPlanner`. Switches the active planner. Request field `planner_type` (uint8) uses the constants defined in the service:

| Constant | ID | Status |
|---|---|---|
| `CLASSIC` | 0 | Implemented (default) |
| `MPC` | 1 | Implemented (closed-loop reactive control) |
| `BATCH` | 2 | **Not implemented** — the call fails |
| `CONSTRAINED` | 3 | **Not implemented** — the call fails (use `trajectory_constraints` on `generate_trajectory` instead) |
| `MULTIPOINT` | 4 | Implemented |
| `JOINT_SPACE` | 5 | Implemented |
| `RETARGET` | 6 | Implemented (IK-based teleoperation follower) |

Response: `success`, `message`, `previous_planner`, `current_planner`.

Planner instances are cached: the first switch to a planner pays its warmup cost, later switches are fast. See [Unified Planner](unified-planner.md).

```bash
ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 1}"
```

### `/unified_planner/get_planners`

Type: `curobo_msgs/srv/GetPlanners` (empty request). Returns `planner_names` (display names), `planner_ids` (matching `SetPlanner` enum IDs), `current_planner_name`, `current_planner_id`, `success`.

### `/unified_planner/clear_trajectory`

Type: `std_srvs/srv/Trigger`. Clears the cached trajectory and the RViz preview.

### `/unified_planner/update_motion_gen_config`

Type: `std_srvs/srv/Trigger`. Rebuilds the motion-planning solver from the current ROS parameters. Call it after changing build-time parameters such as `voxel_size`, `collision_activation_distance`, or `interpolation_dt` at runtime. The rebuild is blocking (roughly 20 s, warmup dominates).

Not needed after `set_collision_cache` — that service rebuilds the solvers itself.

## Services — obstacles and collision world

### `/unified_planner/add_object`

Type: `curobo_msgs/srv/AddObject`. Adds a static obstacle to the scene.

Request:

| Field | Type | Notes |
|---|---|---|
| `type` | `int8` | `CUBOID=0`, `SPHERE=1`, `CAPSULE=2`, `CYLINDER=3`, `MESH=4` |
| `name` | `string` | Must be unique |
| `mesh_file_path` | `string` | Mesh only; absolute path to `.stl`/`.obj` |
| `pose` | `geometry_msgs/Pose` | In the robot base frame |
| `dimensions` | `geometry_msgs/Vector3` | Semantics depend on `type` (see below) |
| `color` | `std_msgs/ColorRGBA` | Visualization only |

Dimension semantics by type:

| Type | `x` | `y` | `z` |
|---|---|---|---|
| Cuboid | full extent X | full extent Y | full extent Z |
| Sphere | radius | — | — |
| Capsule | radius | — | length |
| Cylinder | radius | — | height |
| Mesh | scale X | scale Y | scale Z |

Response: `success`, `message` (e.g. `Object 'table' added ...`).

```bash
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject \
  "{type: 0, name: 'table', pose: {position: {x: 0.6, y: 0.0, z: 0.1}, orientation: {w: 1.0}}, \
    dimensions: {x: 0.8, y: 1.0, z: 0.05}, color: {r: 0.5, g: 0.3, b: 0.1, a: 1.0}}"
```

### `/unified_planner/remove_object`

Type: `curobo_msgs/srv/RemoveObject`. Request: `name`. Removes one obstacle.

### `/unified_planner/remove_all_objects`

Type: `std_srvs/srv/Trigger`. Removes every obstacle. Response message: `All <n> obstacles removed`.

### `/unified_planner/get_obstacles`

Type: `std_srvs/srv/Trigger`. Response `message` lists the current obstacle names, one per line.

### `/unified_planner/attach_object` / `/unified_planner/detach_object`

Types: `curobo_msgs/srv/AttachObject` (request: `object_name`) and `std_srvs/srv/Trigger`. Attaches an existing scene obstacle to the robot flange so it moves with the arm and is collision-checked, then releases it. Requires the robot configuration to define an `attached_object` extra link — see [Tutorial 3](../tutorials/03-collision-objects.md).

### `/unified_planner/set_link_collision`

Type: `curobo_msgs/srv/SetLinkCollision`. Enables or disables the collision spheres of specific robot links (useful when grasping: disable the gripper links against the grasped part). Request: `link_names` (string[]), `enabled` (bool). Response: `success`, `message`, `applied_links`, `unknown_links`. All solvers share the same kinematics configuration, so one call affects planning, MPC, and IK alike. The state persists until the opposite call.

### `/unified_planner/set_collision_spheres_enabled`

Type: `std_srvs/srv/SetBool`. Toggles publication of the robot collision-sphere markers on `/unified_planner/collision_spheres`. **Disabled by default**; enable it for debugging sphere coverage.

### `/unified_planner/get_voxel_grid`

Type: `curobo_msgs/srv/GetVoxelGrid`. Rasterizes the current collision world into a voxel grid.

Request: bounding box in the robot base frame (metres): `bbox_min_x/y/z`, `bbox_max_x/y/z` (typical workspace: −1.52 to 1.52). Response: `voxel_grid` (`nav2_msgs/VoxelGrid`).

### `/unified_planner/get_collision_distance`

Type: `curobo_msgs/srv/GetCollisionDistance` (empty request). Returns the collision distance of each robot sphere at the current joint state: `data` (`float32[]`) and `nb_sphere` (`int8`).

### `/unified_planner/set_collision_cache`

Type: `curobo_msgs/srv/SetCollisionCache`. Resizes the collision caches. Request fields `obb`, `mesh`, `blox` (int32 each): `-1` = leave unchanged, `0` = disable that cache, `>0` = new size. Setting `blox: 0` disables the perception voxel layer — camera-based avoidance stops until it is re-enabled.

Response: `success`, `message`, `obb_cache`, `mesh_cache`, `blox_cache`.

Two behaviors to know:

- The call is **refused while an execution goal is active** (`success: false`) — cancel the goal first.
- On success the solvers are **rebuilt automatically** (blocking, ~20 s). Do not call `update_motion_gen_config` afterwards; the response message ends with `- solvers rebuilt (blocking, ~20s)`.

## Services — kinematics (IK / FK)

IK and FK solvers are **lazy**: they do not exist until you call the matching warmup service. Calling `ik` or `fk` before warmup fails.

### `/unified_planner/warmup_ik` and `/unified_planner/warmup_fk`

Types: `curobo_msgs/srv/WarmupIK`, `curobo_msgs/srv/WarmupFK`. Request: `batch_size` (int32). Builds the solver for that batch size. Calling `ik_batch`/`fk` with a different batch size than the warmup triggers a re-initialization.

### `/unified_planner/ik`

Type: `curobo_msgs/srv/Ik`. Request: `pose` (`geometry_msgs/Pose`, robot base frame). Response: `joint_states` (`sensor_msgs/JointState`), `joint_states_valid` (`std_msgs/Bool`), `error_msg` (`std_msgs/String`), `success`. The IK solve is collision-aware and shares the scene with the motion planner.

### `/unified_planner/ik_batch`

Type: `curobo_msgs/srv/IkBatch`. Same as `ik` with arrays: `poses[]` in, `joint_states[]` / `joint_states_valid[]` out. Poses are solved in parallel on the GPU.

### `/unified_planner/fk`

Type: `curobo_msgs/srv/Fk`. Request: `joint_states` (`sensor_msgs/JointState[]`). Response: `poses` (`geometry_msgs/Pose[]`), `joint_states_valid`, `error_msg`.

See [Tutorial 6](../tutorials/06-ik-fk-services.md) for worked examples.

## Services — robot control strategy

The *control strategy* selects how planned trajectories are sent to a robot (emulator, velocity streaming, position streaming) — see [Tutorial 4](../tutorials/04-robot-execution.md).

### `/unified_planner/set_robot_strategy`

Type: `curobo_msgs/srv/SetRobotStrategy`. Request: `robot_strategy` (string key, e.g. `"emulator"`, `"joint_speed"`, `"joint_pose"`). Response: `success`, `message`, `previous_robot_strategy`, `current_robot_strategy`.

```bash
ros2 service call /unified_planner/set_robot_strategy curobo_msgs/srv/SetRobotStrategy \
  "{robot_strategy: 'emulator'}"
```

### `/unified_planner/get_robot_strategy`

Type: `std_srvs/srv/Trigger`. Returns the active strategy key in `message`.

### `/unified_planner/get_robot_strategies`

Type: `curobo_msgs/srv/GetRobotStrategies` (empty request). Returns `strategy_names` (valid keys), `current_strategy_name`, `success`.

## Action — `/unified_planner/execute_trajectory`

Type: `curobo_msgs/action/SendTrajectory`. The unified "plan (or reuse) and execute" entry point. One goal at a time; a second goal is rejected while one is active.

Goal (mirrors `TrajectoryGeneration`; the active planner decides which fields it consumes):

| Field | Type | Notes |
|---|---|---|
| `start_pose` | `sensor_msgs/JointState` | Empty = current state |
| `target_pose` | `geometry_msgs/Pose` | Classic / MPC / Retarget |
| `target_poses` | `geometry_msgs/Pose[]` | Multi-point |
| `target_joint_positions` | `float64[]` | Joint-space |
| `allow_cached` | `bool` (default `true`) | Reuse a matching, non-expired trajectory from a previous `generate_trajectory` call |

Feedback (status is carried only here — there is no separate status topic):

| Field | Type | Meaning |
|---|---|---|
| `state` | `string` | `PLANNING`, `EXECUTING`, `TRACKING`, `ON_TARGET`, or `FAILED` |
| `step_progression` | `float32` | Open-loop progress, 0 to 1 |
| `position_error` | `float64` | Current Cartesian error (m), reactive planners |
| `on_target` | `bool` | Reactive planners keep servoing after reaching the target |
| `joint_command` | `sensor_msgs/JointState` | Command currently streamed |

Result: `success`, `message`.

With an **open-loop** planner (classic, multi-point, joint-space) the action plans once, streams the trajectory, and succeeds when it ends. With a **closed-loop** planner (MPC, retarget) the action keeps running and servoing; retarget the goal live via `/unified_planner/mpc_goal`, and stop by cancelling the goal.

```bash
ros2 action send_goal /unified_planner/execute_trajectory curobo_msgs/action/SendTrajectory \
  "{target_pose: {position: {x: 0.5, y: 0.2, z: 0.4}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}" --feedback
```

## Topics

### Published by `unified_planner`

| Topic | Type | Rate | Content |
|---|---|---|---|
| `/unified_planner/scene_obstacles` | `visualization_msgs/MarkerArray` | every 2 s | Markers for all scene obstacles |
| `/unified_planner/collision_spheres` | `visualization_msgs/MarkerArray` | every 2 s, **off by default** | Robot collision spheres (enable with `set_collision_spheres_enabled`) |
| `/unified_planner/voxel_grid_sparse` | `curobo_msgs/msg/SparseVoxelGrid` | `sparse_voxel_publish_rate` (default 7 Hz, ≤0 disables) | Occupied voxel indices of the perception layer |
| `/trajectory` | `trajectory_msgs/JointTrajectory` | on plan | Ghost/preview trajectory for RViz |
| `/mpc_predicted_path` | `nav_msgs/Path` | MPC active | Predicted end-effector path over the horizon |
| `/mpc_goal_marker` | `visualization_msgs/Marker` | MPC active | Current MPC goal |
| `/mpc_costs` | `curobo_msgs/msg/MpcCosts` | MPC active | Per-step cost and constraint breakdown |

The active control strategy adds its own topics, configured in the robot descriptor (`robots/<name>.yaml`). With the default Doosan setup: `/leeloo/execute_trajectory` (`trajectory_msgs/JointTrajectory`, published), `/leeloo/trajectory_state` (`std_msgs/Float32`, subscribed), `/dsr01/joint_states` (`sensor_msgs/JointState`, subscribed). The emulator strategy publishes `/emulator/joint_states` (`sensor_msgs/JointState`).

### Subscribed by `unified_planner`

| Topic | Type | Notes |
|---|---|---|
| `/unified_planner/mpc_goal` | `geometry_msgs/Pose` | Live goal retargeting; ignored (with a warning) unless a reactive planner (MPC, retarget) is active |
| depth image topic from `cameras.yaml` | `sensor_msgs/Image` | e.g. `/depth_to_rgb/image_raw`; one subscription per configured camera |
| camera info topic from `cameras.yaml` | `sensor_msgs/CameraInfo` | Read once at startup (5 s wait) when intrinsics are not given in the YAML |

### `robot_segmentation` node

| Interface | Name | Type |
|---|---|---|
| Subscribes | `/depth_to_rgb/image_raw` (param `depth_image_topic`) | `sensor_msgs/Image` |
| Subscribes | `/depth_to_rgb/camera_info` (param `camera_info_topic`) | `sensor_msgs/CameraInfo` |
| Publishes | `/masked_depth_image` | `sensor_msgs/Image` (`16UC1`) |
| Publishes | `/collision_spheres` | `visualization_msgs/MarkerArray` |
| Publishes | `/robot_pointcloud_debug` | `sensor_msgs/PointCloud2` |
| Service | `/set_mask` | `curobo_msgs/srv/SetMask` |
| Service | `/remove_mask` | `curobo_msgs/srv/RemoveObject` |

## Checking node readiness

The planner builds its solvers synchronously at startup (roughly 25–35 s). Two reliable ways to wait for it:

```bash
# The parameter flips to true once warmup completes
ros2 param get /unified_planner node_is_available

# Or wait for the planning service to be advertised
ros2 service list | grep /unified_planner/generate_trajectory
```

There is no `is_available` service — readiness is exposed as the parameter above.

## Related pages

- [Parameters](parameters.md) — every ROS parameter with its default and when it takes effect
- [Unified Planner](unified-planner.md) — planner catalog and switching semantics
- [MPC Implementation](mpc-implementation.md) — the closed-loop reactive control path
- [Tutorial 1: First Trajectory](../tutorials/01-first-trajectory.md)
