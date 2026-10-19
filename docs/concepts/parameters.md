# Parameters Guide

Every parameter of the `unified_planner` node, with its real default, what it does, and when a change takes effect. All of them come from one schema, `curobo_ros/core/parameters.py`, which declares them at node start with a `ParameterDescriptor`: `ros2 param describe /unified_planner <name>` shows the same description, and the *when* in `additional_constraints`. If this page and the schema ever disagree, the schema wins (`test/test_parameters.py` checks that every schema entry is listed here).

## How parameters take effect

Each parameter has one of four kinds:

| Kind | How to change | When it takes effect |
|---|---|---|
| **Startup** | Launch file / `--ros-args -p` only. Declared `read_only`: `ros2 param set` is refused | Restart |
| **Build** | `ros2 param set /unified_planner <name> <value>` | When the solver that reads it is built. `MotionPlanner`: `update_motion_gen_config` (blocking, ~20 s). MPPI / L-BFGS / retargeter: first switch to that planner; once built, only a `set_collision_cache` rebuild or a restart applies it. The node logs a warning naming the solvers that keep the old value |
| **Live** | `ros2 param set …` | Next use (request, goal or cycle) |
| **Status** | Written by the node | — |

Robot speed is not a parameter: set the cspace velocity/acceleration/jerk limits in the cuRobo robot YAML, then rebuild with `update_motion_gen_config`.

At startup the node warns about every launch override that no parameter consumes (`Parameter 'x' is set but not used by this node: IGNORED`), and names the replacement of removed ones (`decay_factor`, `mpc_solver_type`).

## Launch arguments (`gen_traj.launch.py`)

These are the launch arguments that actually configure the system:

| Argument | Default | Description |
|---|---|---|
| `robot` | `doosan_m1013` | Robot descriptor name — loads `robots/<name>.yaml` (e.g. `doosan_m1013`, `emulator`) |
| `robot_config_file` | `''` (auto) | cuRobo robot YAML; empty = resolved from the robot descriptor |
| `urdf_path` | `''` (auto) | URDF for `robot_state_publisher`; empty = read from the robot descriptor |
| `cameras_config_file` | `''` | Camera configuration YAML (see [Tutorial 7](../tutorials/07-pointcloud-detection.md)); empty = no cameras |
| `world_file` | `''` | Static world YAML (e.g. `config/floor_world.yml`); empty = empty world — **including no floor** |
| `gui` | `true` | Start RViz |
| `voxel_size` | `0.05` | Perception/collision voxel size (m). Startup only |
| `mapper_extent_xyz` | `[2.56, 2.56, 2.56]` | Perception volume extent (m), centred on the robot base |
| `max_attempts` | `1` | Planning retries per request |
| `time_dilation_factor` | `0.5` | Feedback publish period (s) during open-loop execution |
| `collision_activation_distance` | `0.025` | Distance (m) at which the collision cost activates |

`voxel_size`, `max_attempts`, `time_dilation_factor` and `collision_activation_distance` are forwarded to the node with its own defaults. `mapper_extent_xyz` overrides the node default (`[2.0, 2.0, 2.0]`) with 2.56 m, i.e. 128 voxels of 0.02 m. See the tables below for what each one does.

There is no world floor added automatically at startup: if you want a ground plane, pass a `world_file` that contains one (the shipped `config/floor_world.yml` does).

## Node parameters — Planning

| Parameter | Default | Effect | Kind |
|---|---|---|---|
| `planner_type` | `'classic'` | Planner selected at startup (classic, mpc, lbfgs, multi_point, joint_space, retarget). Switch at runtime with set_planner | Startup |
| `max_attempts` | `1` | Planning retries per request. Range 1–1000 | Live |
| `interpolation_dt` | `0.025` | Time step (s) of every output trajectory. Must match the robot driver's command period. Range 0.001–1.0. The command strategies re-read it at set_robot_strategy | Build (MotionPlanner, MPPI, L-BFGS) |
| `collision_activation_distance` | `0.025` | Distance (m) at which the collision cost activates. Range 0.0–1.0. get_collision_distance reads it on each call | Build (MotionPlanner) |
| `use_cuda_graph` | `true` | Capture/replay CUDA graphs in the solvers. CUROBO_USE_CUDA_GRAPH=0 overrides it | Startup |
| `time_dilation_factor` | `0.5` | Feedback publish period (s) during open-loop execution. Not a speed control: set speed in the robot YAML cspace | Live |
| `trajectory_cache_ttl` | `30.0` | Lifetime (s) of a trajectory cached by generate_trajectory | Live |
| `push_esdf_to_solvers` | `true` | Diagnostic: false withholds the camera ESDF from the solvers (disables camera-based avoidance) | Live |

## Node parameters — Perception (Mapper)

| Parameter | Default | Effect | Kind |
|---|---|---|---|
| `voxel_size` | `0.05` | Voxel size (m) of the perception ESDF and the voxel collision cache; default resolution of get_voxel_grid. Range 0.001–1.0 | Startup |
| `sparse_voxel_publish_rate` | `7.0` | Publish rate (Hz) of voxel_grid_sparse. <= 0 disables it | Startup |
| `use_mapper` | `true` | Enable the cuRobo Mapper (camera ESDF perception) | Startup |
| `mapper_extent_xyz` | `[2.0, 2.0, 2.0]` | Perception volume (m) around mapper_grid_center | Startup |
| `mapper_grid_center` | `[0.0, 0.0, 0.0]` | Centre of the perception volume, in the robot base frame | Startup |
| `mapper_voxel_size` | `0.02` | Internal TSDF voxel size (m). Range 0.001–1.0 | Startup |
| `mapper_image_width` | `640` | Depth-projection width: every depth frame is resized to it | Startup |
| `mapper_image_height` | `480` | Depth-projection height: every depth frame is resized to it | Startup |
| `mapper_depth_min` | `0.1` | Near depth clipping (m) | Startup |
| `mapper_depth_max` | `5.0` | Far depth clipping (m) | Startup |
| `decay_half_life_s` | `0.7` | Time (s) for an unobserved voxel to lose half its TSDF weight. <= 0 disables the decay | Startup |

## Node parameters — Reactive control (all closed-loop planners)

| Parameter | Default | Effect | Kind |
|---|---|---|---|
| `convergence_threshold` | `0.01` | End-effector position error (m) considered on target | Live |
| `convergence_threshold_rad` | `0.05` | End-effector orientation error (rad) considered on target | Live |
| `convergence_hold_steps` | `1` | Consecutive in-tolerance steps before on_target goes true. Range 1–100000 | Live |
| `max_mpc_iterations` | `1000` | Safety cap on servo steps per goal | Live |
| `mpc_debug` | `false` | Per-step diagnostic CSVs and [MPC DIAG] logs | Live |
| `lbfgs_debug` | `false` | Perception timing logs (node-wide; unrelated to lbfgs.lbfgs_debug) | Live |
| `diagnostic_csv_dir` | `''` | Directory of the diagnostic CSVs. Empty = the ROS log directory | Live |

## Node parameters — MPPI (planner `mpc`)

| Parameter | Default | Effect | Kind |
|---|---|---|---|
| `mpc_step_dt` | `0.03` | MPPI optimization time step (s). Range 0.001–1.0 | Build (MPPI) |
| `mpc_horizon_steps` | `30` | MPPI receding horizon length. Range 1–10000 | Build (MPPI) |
| `mpc_warm_start_iters` | `5` | MPPI iterations per solve after the first. Range 1–100000 | Build (MPPI) |
| `mpc_cold_start_iters` | `10` | MPPI iterations on the first solve. Range 1–100000 | Build (MPPI) |
| `mpc_mppi_num_particles` | `400` | MPPI particle count. Range 1–1000000 | Build (MPPI) |
| `mpc_vel_feedback_alpha` | `1.0` | Blend of the measured velocity into the MPPI state. Range 0.0–1.0 | Build (MPPI) |
| `mpc_command_interval` | `0.12` | MPPI command pacing (s): each window executes before re-solving. 0 = re-solve as fast as possible. Re-read at each goal | Live |

## Node parameters — L-BFGS (planner `lbfgs`)

| Parameter | Default | Effect | Kind |
|---|---|---|---|
| `lbfgs_apply_custom_config` | `false` | Use the lbfgs.* cost/optimizer tree instead of cuRobo's stock config | Build (L-BFGS) |
| `lbfgs_warm_start_iters` | `0` | L-BFGS iterations per solve after the first; multiple of 25. 0 = cuRobo default. Range 0–100000 | Build (L-BFGS) |
| `lbfgs_cold_start_iters` | `0` | L-BFGS iterations on the first solve; multiple of 25. 0 = cuRobo default. Range 0–100000 | Build (L-BFGS) |
| `lbfgs_command_points` | `6` | Points per publish; n-1 are executed, the last is a spare (min 2) | Build (L-BFGS) |
| `lbfgs_publish_points` | `11` | Points actually published (>= lbfgs_command_points); the extra tail only plays when a solve runs late. Re-read at each goal | Live |
| `lbfgs_dynamic_m` | `true` | Count executed points from the measured publish cycle, not the nominal one | Build (L-BFGS) |
| `lbfgs_profile_sync` | `false` | Debug: synchronize CUDA at every phase boundary (slows the loop) | Build (L-BFGS) |

## Node parameters — Retargeting (planner `retarget`)

| Parameter | Default | Effect | Kind |
|---|---|---|---|
| `retarget_position_weight` | `1.0` | Retarget position weight | Build (retargeter) |
| `retarget_orientation_weight` | `1.0` | Retarget orientation weight | Build (retargeter) |
| `retarget_time_constant_s` | `0.3` | Retarget velocity command: joint error / this time constant (s). Smaller = snappier, more overshoot. Range 0.01–10.0 | Live |
| `retarget_max_joint_speed_dps` | `30.0` | Retarget velocity command: per-joint speed limit (deg/s). Range 0.1–360.0 | Live |
| `retarget_use_mpc` | `false` | Follow frames after the first with the MPC sub-solver instead of local IK | Build (retargeter) |

## Node parameters — Robot, world and command strategies

| Parameter | Default | Effect | Kind |
|---|---|---|---|
| `robot` | `'doosan_m1013'` | Robot descriptor: robots/<name>.yaml or a path (model, driver topics, default strategy) | Startup |
| `robot_config_file` | from descriptor | cuRobo robot YAML. Empty = the one resolved from the robot descriptor | Startup |
| `base_link` | from descriptor | Robot base frame (default from the robot descriptor) | Startup |
| `world_file` | `''` | Static world YAML loaded at startup. Empty = empty world, no floor | Startup |
| `cameras_config_file` | `''` | Camera configuration YAML. Empty = no cameras | Startup |
| `control_strategy` | from descriptor | How commands reach the robot (emulator, joint_speed, joint_pose). Sets the startup strategy; change it at runtime with set_robot_strategy | Status |
| `max_joint_accel_dps` | from descriptor | joint_speed strategy: joint acceleration clamp (deg/s^2) against the measured velocity | Live |
| `joint_speed_debug_csv` | `false` | joint_speed strategy: per-publish diagnostic CSV | Live |
| `node_is_available` | `false` | True once the startup warmup has completed | Status |

## `robot_segmentation` node parameters

| Parameter | Default | Effect |
|---|---|---|
| `depth_image_topic` | `/depth_to_rgb/image_raw` | Input depth image |
| `camera_info_topic` | `/depth_to_rgb/camera_info` | Camera intrinsics |
| `robot_base_frame` | `base_0` | TF frame of the robot base |
| `mask_margin` | `0.0` | Extra margin (m) around the robot mask |
| `distance_threshold` | `0.05` | Distance (m) to a collision sphere below which a pixel is masked |
| `payload_distance_threshold` | `0.02` | Same, for the spheres of an attached payload only. Tighter on purpose: those spheres were just fitted to a supplied geometry, and a 5 cm halo would erase the table at the moment of placing onto it |
| `attached_spheres_topic` | `/unified_planner/attached_spheres` | Planner broadcast of the attached payload's spheres. Set it if the planner node was renamed; empty string disables the subscription |

## Robot YAML configuration

The cuRobo robot configuration (for the Doosan M1013: `curobo_doosan/src/m1013/m1013.yml`) defines kinematics (`urdf_path`, `base_link`, `ee_link`), the cspace with joint limits (position/velocity/acceleration/jerk), and the collision spheres. This is also where the robot's *speed* is set — scale the cspace velocity/acceleration/jerk limits and rebuild with `update_motion_gen_config`.

See [Tutorial 2: Adding Your Robot](../tutorials/02-adding-your-robot.md) for the full anatomy.

## Tuning guidance

**`voxel_size`** is the main precision/VRAM trade-off. It sizes the Mapper's ESDF and the voxel collision cache at startup, so it is a launch argument (`voxel_size:=0.03`), not a runtime knob. `0.05` is a good default; `0.02–0.03` for cluttered scenes on GPUs with headroom; `0.1` for large sparse environments. Smaller voxels cube the memory cost. To measure the cost on your own GPU before committing, run the shipped benchmark against a live planner:

```bash
ros2 run curobo_ros benchmark_voxel_grid --ros-args -p voxel_size:=0.02 -p n_runs:=10
```

**`collision_activation_distance`** adds safety margin at the cost of a smaller reachable workspace in clutter. `0.025` default; raise toward `0.05–0.1` around humans, lower toward `0.01` for tight picking.

**Collision caches** (`set_collision_cache` service, fields `obb`/`mesh`/`blox`) bound how many obstacles of each kind the solvers can hold. Raise `obb` if adding objects fails with a cache error; set `blox: 0` only if you deliberately want to disable camera perception. Every change triggers an automatic ~20 s solver rebuild and is refused while an execution goal is active.

**MPC**: the recipe follows the planner (`set_planner` `mpc` = MPPI in acceleration space, validated on the real M1013; `lbfgs` = L-BFGS + B-spline). `mpc_warm_start_iters`/`mpc_cold_start_iters` (5/10) tune MPPI only; L-BFGS has its own `lbfgs_warm_start_iters`/`lbfgs_cold_start_iters`, multiples of 25 (e.g. 25/100). Set reactive build parameters **before** the first switch to that planner.

## Worked example

```bash
# Finer voxel grid: a startup parameter, set at launch
ros2 launch curobo_ros gen_traj.launch.py voxel_size:=0.03

# Wider collision margin for the MotionPlanner: build parameter, then rebuild
ros2 param set /unified_planner collision_activation_distance 0.05
ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger

# Give planning more retries (live, no rebuild)
ros2 param set /unified_planner max_attempts 3

# Grow the OBB cache (rebuilds by itself, ~20 s)
ros2 service call /unified_planner/set_collision_cache curobo_msgs/srv/SetCollisionCache \
  "{obb: 300, mesh: -1, blox: -1}"

# What a parameter does and when it applies
ros2 param describe /unified_planner mpc_horizon_steps
```

## Related pages

- [ROS Interfaces](ros-interfaces.md) — the services referenced above
- [MPC Implementation](mpc-implementation.md) — what the `mpc_*` family controls
- [Tutorial 1: First Trajectory](../tutorials/01-first-trajectory.md)
