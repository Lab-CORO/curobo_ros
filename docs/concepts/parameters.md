# Parameters Guide

This guide explains all important parameters in curobo_ros, how they affect performance, and how to tune them for your application.

---

## Overview

curobo_ros has two types of parameters:

| Type | Description | How to Change |
|------|-------------|---------------|
| **Launch Parameters** | Set when launching the node | Pass as arguments to `ros2 launch` |
| **Runtime Parameters** | Can be changed while node is running | Use `ros2 param set` command |
| **Configuration Parameters** | Defined in robot YAML files | Edit config file + reload configuration |

---

## Launch File Parameters

The `gen_traj.launch.py` file accepts several parameters that control the behavior of the trajectory generation system:

### Launch Parameters Reference

| Parameter | Type | Default | Description | Required |
|-----------|------|---------|-------------|----------|
| `robot_config_file` | string | m1013.yml | Path to robot configuration YAML file | No |
| `urdf_path` | string | "" (auto) | Path to URDF file (if empty, loaded from robot_config_file) | No |
| `cameras_config_file` | string | "" | Path to cameras configuration YAML file | No |
| `world_file` | string | "" | Path to world configuration YAML file | No |
| `gui` | bool | true | Launch RViz visualization | No |
| `include_realsense_launch` | bool | false | Include RealSense camera launch | No |
| `max_attempts` | int | 2 | Maximum planning attempts | No |
| `timeout` | float | 1.0 | Planning timeout in seconds | No |
| `time_dilation_factor` | float | 0.5 | Time dilation for trajectory execution | No |
| `voxel_size` | float | 1.0 | Voxel size for collision checking | No |
| `collision_activation_distance` | float | 0.5 | Distance for collision activation | No |

### Usage Examples

```bash
# Basic launch with custom robot config
ros2 launch curobo_ros gen_traj.launch.py robot_config_file:=$(ros2 pkg prefix curobo_doosan)/config/m1013.yml

# Launch without GUI (headless mode)
ros2 launch curobo_ros gen_traj.launch.py gui:=false

# Launch with custom parameters
ros2 launch curobo_ros gen_traj.launch.py \
    voxel_size:=0.05 \
    max_attempts:=5 \
    timeout:=2.0
```

## Critical Parameters

These two parameters have the **biggest impact** on system behavior:

### 1. `voxel_size` (meters)

**What it does**: Defines the size of voxels (3D grid cells) used for collision checking.

**Type**: Launch parameter (requires config reload to change)

**Default**: `0.05` m (5 cm)

**Range**: `0.01` - `0.2` m

#### How It Works

The environment is discretized into a 3D grid of voxels. Each voxel is either:
- **Occupied** (contains an obstacle)
- **Free** (safe space)

The robot is represented as spheres, and collision checking tests if any sphere intersects occupied voxels.

#### Impact on Performance

| Voxel Size | Resolution | Planning Speed | Accuracy | GPU Memory | Use Case |
|------------|-----------|----------------|----------|------------|----------|
| **0.01 m** (1 cm) | Very High | ⚠️ Slow | ✅ Excellent | ⚠️ High | Fine manipulation, small objects |
| **0.02 m** (2 cm) | High | 🟡 Medium | ✅ Good | 🟡 Medium | Precision tasks |
| **0.05 m** (5 cm) | Medium | ✅ Fast | 🟡 Good | ✅ Low | General planning (default) |
| **0.1 m** (10 cm) | Low | ✅ Very Fast | ⚠️ Poor | ✅ Very Low | Fast planning, large clearances |

#### Trade-offs

**Smaller voxels (0.01 - 0.02 m)**:
- ✅ More accurate obstacle representation
- ✅ Can navigate tight spaces
- ✅ Better for small objects (screws, cables, thin obstacles)
- ❌ Slower planning (more voxels to check)
- ❌ Higher GPU memory usage
- ❌ May fail to plan in cluttered environments (too conservative)

**Larger voxels (0.05 - 0.1 m)**:
- ✅ Faster planning (fewer voxels)
- ✅ Lower GPU memory usage
- ✅ More robust in cluttered environments
- ❌ Less accurate obstacle representation
- ❌ May miss small obstacles
- ❌ Larger safety margins (less efficient paths)

#### Recommended Values

| Application | Recommended `voxel_size` | Reason |
|-------------|-------------------------|--------|
| Pick and place (small parts) | 0.02 m | Accurate detection of small objects |
| Assembly tasks | 0.02 - 0.03 m | Balance between speed and precision |
| General manipulation | 0.05 m | Good balance (default) |
| Fast motion in open spaces | 0.08 - 0.1 m | Maximum speed |
| Dense point cloud environments | 0.03 - 0.05 m | Handle high point density |

#### How to Set

```bash
# At launch time
ros2 launch curobo_ros gen_traj.launch.py voxel_size:=0.02

# To change at runtime (requires reload)
ros2 param set /unified_planner voxel_size 0.02
ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger
```

#### Visual Guide

```
Voxel Size = 0.01m (1cm)        Voxel Size = 0.05m (5cm)
┌─┬─┬─┬─┬─┬─┬─┬─┐              ┌────┬────┐
├─┼─┼█┼█┼█┼─┼─┼─┤              │    │████│
├─┼─┼█┼█┼█┼─┼─┼─┤              │    │████│
├─┼─┼█┼█┼█┼─┼─┼─┤              └────┴────┘
├─┼─┼─┼─┼─┼─┼─┼─┤
└─┴─┴─┴─┴─┴─┴─┴─┘
High resolution (slow)           Low resolution (fast)
Detects small obstacles          May miss small obstacles
```

---

### 2. `time_dilation_factor` (ratio)

**What it does**: Scales the time duration of generated trajectories, effectively controlling robot speed.

**Type**: Runtime parameter (can be changed on the fly)

**Default**: `0.5` (50% of maximum speed)

**Range**: `0.0` - `1.0` (typically use `0.3` - `1.0`)

#### How It Works

cuRobo plans trajectories based on joint velocity and acceleration limits defined in your robot's URDF/config. The `time_dilation_factor` stretches or compresses the time axis:

```
Original trajectory: 2.0 seconds
time_dilation_factor = 0.5 → Executed in 4.0 seconds (slower, safer)
time_dilation_factor = 1.0 → Executed in 2.0 seconds (faster, at limits)
```

**Important**: This is NOT a simple speed multiplier. It affects:
- Trajectory duration
- Velocity profiles
- Acceleration profiles
- Smoothness

#### Impact on Behavior

| Factor | Speed | Safety | Smoothness | Real-time Control Margin | Use Case |
|--------|-------|--------|------------|-------------------------|----------|
| **0.3** | Very Slow | ✅ Very Safe | ✅ Very Smooth | ✅ Large | Testing, debugging |
| **0.5** | Moderate | ✅ Safe | ✅ Smooth | ✅ Good | Normal operation (default) |
| **0.7** | Fast | 🟡 Moderate | 🟡 Good | 🟡 Medium | Production speed |
| **1.0** | Maximum | ⚠️ At Limits | ⚠️ Aggressive | ⚠️ Tight | Time-critical tasks |

#### Trade-offs

**Lower values (0.3 - 0.5)**:
- ✅ Slower, more controlled motion
- ✅ More time for real-time controller to react
- ✅ Smoother trajectories
- ✅ Better for testing and debugging
- ✅ Safer (more margin for error)
- ❌ Longer cycle times
- ❌ Lower productivity

**Higher values (0.7 - 1.0)**:
- ✅ Faster execution
- ✅ Higher productivity
- ✅ Shorter cycle times
- ❌ Less time for controller to react
- ❌ More aggressive motion
- ❌ Higher risk of tracking errors
- ❌ May violate constraints if robot is not well-tuned

#### Recommended Values

| Scenario | Recommended Factor | Reason |
|----------|-------------------|--------|
| First tests / debugging | 0.3 - 0.4 | Very safe, easy to stop |
| Development | 0.4 - 0.5 | Good balance for iteration |
| Normal operation | 0.5 - 0.6 | Default safe speed |
| Production (well-tested) | 0.6 - 0.8 | Optimized cycle time |
| Time-critical (expert only) | 0.8 - 1.0 | Maximum speed |
| Teaching/demonstration | 0.3 - 0.4 | Slow enough to observe |

#### How to Set

```bash
# At launch time
ros2 launch curobo_ros gen_traj.launch.py time_dilation_factor:=0.7

# At runtime (takes effect on next trajectory)
ros2 param set /unified_planner time_dilation_factor 0.7
```

#### Example

```python
# Python example: Dynamic speed adjustment
import rclpy
from rclpy.node import Node

node = Node('speed_controller')

# Start slow for testing
node.set_parameters([rclpy.parameter.Parameter('time_dilation_factor',
                                                rclpy.Parameter.Type.DOUBLE,
                                                0.3)])

# ... test trajectory ...

# Increase speed once confident
node.set_parameters([rclpy.parameter.Parameter('time_dilation_factor',
                                                rclpy.Parameter.Type.DOUBLE,
                                                0.7)])
```

---

## Other Important Parameters

### 3. `collision_activation_distance` (meters)

**What it does**: Distance threshold at which collision avoidance activates.

**Type**: Launch parameter (requires config reload)

**Default**: `0.025` m (2.5 cm)

**Range**: `0.01` - `0.1` m

#### How It Works

The robot will start avoiding obstacles when it gets within this distance. Think of it as a "safety bubble" around the robot.

#### Impact

| Distance | Behavior | Use Case |
|----------|----------|----------|
| **0.01 m** (1 cm) | Tight clearances, risky | Pick and place in dense environments |
| **0.025 m** (2.5 cm) | Balanced (default) | General use |
| **0.05 m** (5 cm) | Conservative, large margins | Safety-critical applications |

**Trade-off**: Smaller values allow tighter paths but increase collision risk. Larger values are safer but may fail to find paths in constrained spaces.

#### How to Set

```bash
ros2 launch curobo_ros gen_traj.launch.py collision_activation_distance:=0.03
```

---

### 4. `max_attempts` (integer)

**What it does**: Maximum number of planning attempts before giving up.

**Type**: Runtime parameter

**Default**: `1`

**Range**: `1` - `10`

#### How It Works

If planning fails, the node will retry up to `max_attempts` times with different random seeds.

#### When to Increase

- Planning often fails on the first try (complex environments)
- You need higher success rates
- Execution time is not critical

**Note**: Each attempt adds planning time (~500ms per attempt).

#### How to Set

```bash
# At launch
ros2 launch curobo_ros gen_traj.launch.py max_attempts:=3

# At runtime
ros2 param set /unified_planner max_attempts 3
```

---

### 5. `timeout` (seconds)

**What it does**: Maximum time to spend planning a single trajectory.

**Type**: Runtime parameter

**Default**: `5.0` seconds

**Range**: `0.5` - `30.0` seconds

#### How It Works

If planning takes longer than `timeout`, it will fail and return an error.

#### Tuning Guidelines

| Timeout | Use Case |
|---------|----------|
| **0.5 - 1.0 s** | Real-time applications, simple environments |
| **2.0 - 5.0 s** | Normal operation (default) |
| **10.0 - 30.0 s** | Complex environments, difficult planning problems |

**Trade-off**: Longer timeouts increase success rate but reduce responsiveness.

#### How to Set

```bash
# At runtime
ros2 param set /unified_planner timeout 2.0
```

---

### 6. `robot_config_file` (path)

**What it does**: Path to the robot configuration YAML file.

**Type**: Launch parameter

**Default**: `m1013.yml` (Doosan M1013)

#### What's in the Config File

- Robot kinematics (URDF path, base_link, ee_link)
- Joint limits (position, velocity, acceleration)
- Collision spheres (robot self-collision model)
- CSpace parameters

#### How to Set

```bash
ros2 launch curobo_ros gen_traj.launch.py robot_config_file:=path/to/your_robot.yml
```

See [Adding Your Robot Tutorial](../tutorials/02-adding-your-robot.md) for details.

---

## Collision Cache Parameters

### What is Collision Cache?

The **collision cache** defines GPU memory allocation for different collision checking methods. cuRobo supports three types of collision geometry, each with its own cache:

1. **OBB Cache** - Oriented Bounding Boxes (cuboids, fused mesh cuboids)
2. **Mesh Cache** - High-fidelity mesh representations
3. **Blox Cache** - Voxelized representations (point clouds, depth images)

**Critical Concept**: The cache size determines how many collision objects can be stored in GPU memory simultaneously. Exceeding the cache will cause planning failures.

### The Three Collision Caches

#### 1. OBB Cache (Oriented Bounding Boxes)

**What it stores:**
- Cuboid primitives added via `add_object` with `primitive_type: 0`
- **Fused cuboids generated from mesh voxelization** (most common use)

**Default Size:** `100`

**When to increase:**
- Adding many cuboid objects (>50)
- **Adding mesh objects** (meshes are converted to multiple cuboids)
- Error: "OBB cache exceeded"

**Example - Mesh requires OBB cache:**
```bash
# Add mesh object
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject \
  "{name: 'gear', primitive_type: 4, mesh_file_path: '/path/to/gear.stl', \
    pose: {position: {x: 0.5, y: 0.0, z: 0.3}, orientation: {w: 1.0}}}"

# Error: OBB cache exceeded!
# Solution: Increase OBB cache
ros2 service call /unified_planner/set_collision_cache \
  curobo_msgs/srv/SetCollisionCache "{obb: 300, mesh: -1, blox: -1}"

# Apply changes
ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger
```

**Why meshes need OBB cache:**
Internally, meshes are voxelized into multiple cuboids for efficient collision checking. A single mesh might generate 50-200 cuboids depending on:
- Mesh complexity
- Voxel size (smaller voxel_size = more cuboids)
- Mesh dimensions

#### 2. Mesh Cache

**What it stores:**
- High-fidelity mesh representations for collision checking
- Used when `collision_checker_type` is set to `MESH`

**Default Size:** Not allocated by default

**When to use:**
- High-precision collision checking with complex geometry
- When voxelization is too coarse
- **Note**: MESH collision checker is slower than BLOX

**Typical Usage:**
```bash
# Allocate mesh cache (if using MESH collision checker)
ros2 service call /unified_planner/set_collision_cache \
  curobo_msgs/srv/SetCollisionCache "{obb: -1, mesh: 50, blox: -1}"

ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger
```

**Important**: Most users should use the default BLOX collision checker and not allocate mesh cache. Only use MESH collision checker for specialized high-precision applications.

#### 3. Blox Cache

**What it stores:**
- Voxelized collision representations
- Point cloud data from cameras
- Depth image data
- Pre-voxelized environments

**Default Size:** `10`

**When to increase:**
- Using point cloud collision checking
- Adding voxel grid representations
- Multiple depth cameras
- Error: "Blox cache exceeded"

**Example - Point cloud usage:**
```bash
# For point cloud-based collision avoidance
ros2 service call /unified_planner/set_collision_cache \
  curobo_msgs/srv/SetCollisionCache "{obb: -1, mesh: -1, blox: 50}"

ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger
```

### SetCollisionCache Service

**Service Name:** `/unified_planner/set_collision_cache`
**Service Type:** `curobo_msgs/srv/SetCollisionCache`

**Request Parameters:**
```
int32 obb    # OBB cache size (-1 = no change)
int32 mesh   # Mesh cache size (-1 = no change)
int32 blox   # Blox cache size (-1 = no change)
```

**Important**: Use `-1` to keep existing cache size unchanged.

**Example - Increase only OBB cache:**
```bash
# Only modify OBB cache, keep mesh and blox unchanged
ros2 service call /unified_planner/set_collision_cache \
  curobo_msgs/srv/SetCollisionCache "{obb: 200, mesh: -1, blox: -1}"

# CRITICAL: Apply changes
ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger
```

### CRITICAL: update_motion_gen_config Required

**After calling `set_collision_cache`, you MUST call `update_motion_gen_config` to apply changes.**

```bash
# Pattern for updating collision cache
ros2 service call /unified_planner/set_collision_cache \
  curobo_msgs/srv/SetCollisionCache "{obb: 300, mesh: -1, blox: -1}"

# This call is REQUIRED
ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger
```

**Why**: The collision cache is part of the MotionGen configuration. Changes are staged in memory but not applied until `update_motion_gen_config` reloads the planner configuration.

### Mesh Handling and Collision Cache

**Key Concept**: When you add a mesh object using `add_object`, cuRobo stores it in **TWO** places:

1. **world_cfg.mesh** - Original high-fidelity mesh
2. **world_cfg.cuboid** - Voxelized cuboid approximations (for BLOX checker)

**The voxelized cuboids consume OBB cache, NOT mesh cache.**

#### How Mesh Voxelization Works

```bash
# 1. Add mesh object
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject \
  "{name: 'part', primitive_type: 4, mesh_file_path: '/path/to/part.stl', \
    pose: {position: {x: 0.5, y: 0.0, z: 0.3}, orientation: {w: 1.0}}}"
```

**Internal Process:**
1. Load mesh from file
2. Voxelize mesh using MeshBloxilization pipeline
3. Fuse voxels into cuboids (e.g., 50-150 cuboids)
4. Store original mesh in `world_cfg.mesh`
5. Store cuboids in `world_cfg.cuboid` (named `part_cuboid_0`, `part_cuboid_1`, ...)
6. Track mapping in `mesh_cuboid_mapping`

**Result**: The mesh uses OBB cache for the generated cuboids.

#### Estimating OBB Cache Needs for Meshes

| Mesh Complexity | Voxel Size | Estimated Cuboids | Recommended OBB Cache |
|----------------|------------|-------------------|----------------------|
| Simple (box, cylinder) | 0.05 m | 10-30 | 150 |
| Moderate (gear, bracket) | 0.05 m | 30-80 | 200 |
| Complex (engine part) | 0.05 m | 80-200 | 300-500 |
| Very complex (assembly) | 0.05 m | 200-500 | 600-1000 |

**Note**: Smaller `voxel_size` values generate MORE cuboids per mesh.

#### Example: Adding Multiple Meshes

```bash
# Scenario: Adding 3 complex meshes to scene

# 1. Increase OBB cache (3 meshes × ~100 cuboids each + buffer)
ros2 service call /unified_planner/set_collision_cache \
  curobo_msgs/srv/SetCollisionCache "{obb: 400, mesh: -1, blox: -1}"

ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger

# 2. Add first mesh
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject \
  "{name: 'gear1', primitive_type: 4, mesh_file_path: '/path/to/gear.stl', \
    pose: {position: {x: 0.5, y: 0.0, z: 0.3}, orientation: {w: 1.0}}}"

# 3. Add second mesh
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject \
  "{name: 'gear2', primitive_type: 4, mesh_file_path: '/path/to/gear.stl', \
    pose: {position: {x: 0.5, y: 0.3, z: 0.3}, orientation: {w: 1.0}}}"

# 4. Add third mesh
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject \
  "{name: 'bracket', primitive_type: 4, mesh_file_path: '/path/to/bracket.stl', \
    pose: {position: {x: 0.3, y: 0.0, z: 0.2}, orientation: {w: 1.0}}}"

# 5. Verify obstacles added
ros2 service call /unified_planner/get_obstacles std_srvs/srv/Trigger

# Output:
# message: "gear1, gear2, bracket"
```

### Relationship Between Parameters

Several parameters interact with collision cache:

#### voxel_size affects OBB cache needs

```bash
# Smaller voxel_size = more cuboids per mesh = higher OBB cache needed

# Example with large voxels (fewer cuboids)
ros2 param set /unified_planner voxel_size 0.08
ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger

# Can use smaller OBB cache
ros2 service call /unified_planner/set_collision_cache \
  curobo_msgs/srv/SetCollisionCache "{obb: 150, mesh: -1, blox: -1}"
ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger

# Example with small voxels (more cuboids)
ros2 param set /unified_planner voxel_size 0.02
ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger

# Need larger OBB cache
ros2 service call /unified_planner/set_collision_cache \
  curobo_msgs/srv/SetCollisionCache "{obb: 500, mesh: -1, blox: -1}"
ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger
```

#### collision_checker_type affects which cache is used

```bash
# BLOX checker (default) - uses OBB and Blox caches
# Most efficient, recommended for most applications

# MESH checker - uses Mesh cache
# Higher precision, slower, rarely needed
```

### Viewing Current Cache Settings

```bash
# Check collision_cache parameter
ros2 param get /unified_planner collision_cache

# Output example:
# {'obb': 200, 'mesh': 0, 'blox': 10}
```

### Collision Cache Best Practices

#### 1. Start with Conservative Sizes

```bash
# For scenes with meshes, start with larger OBB cache
ros2 service call /unified_planner/set_collision_cache \
  curobo_msgs/srv/SetCollisionCache "{obb: 300, mesh: -1, blox: -1}"

ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger
```

#### 2. Monitor for Cache Exceeded Errors

Watch for errors like:
- "OBB cache exceeded"
- "Blox cache exceeded"
- "Mesh cache exceeded"

When you see these, increase the relevant cache size.

#### 3. Balance Cache Size vs GPU Memory

**Larger cache = more GPU memory used**

Each cache type has different memory footprints:
- **OBB**: ~100 bytes per entry
- **Mesh**: ~1-10 KB per mesh (depends on mesh complexity)
- **Blox**: ~1-50 MB per voxel grid (depends on voxel_size and grid dimensions)

**Recommendations:**
- **GPU with 4-6 GB VRAM**: OBB=200, Mesh=0, Blox=20
- **GPU with 8-12 GB VRAM**: OBB=500, Mesh=0, Blox=50
- **GPU with 16+ GB VRAM**: OBB=1000, Mesh=50, Blox=100

#### 4. Adjust Cache Based on Scene Complexity

| Scene Type | OBB Cache | Mesh Cache | Blox Cache |
|------------|-----------|------------|------------|
| Simple (few cuboids, no meshes) | 100 | 0 | 10 |
| Moderate (1-3 meshes) | 200-300 | 0 | 10 |
| Complex (5-10 meshes) | 500-800 | 0 | 10 |
| With point clouds | 150 | 0 | 50 |
| High-precision with mesh checker | 150 | 50 | 10 |

#### 5. Remove Unused Objects

```bash
# Remove objects to free up cache
ros2 service call /unified_planner/remove_object curobo_msgs/srv/RemoveObject \
  "{name: 'old_mesh'}"

# This frees up OBB cache used by the mesh's cuboids
```

### Troubleshooting Collision Cache Issues

#### Issue 1: "OBB cache exceeded" Error

**Symptoms:**
```
Error: OBB cache exceeded, cannot add object
Failed to add object 'my_mesh'
```

**Solution:**
```bash
# Increase OBB cache
ros2 service call /unified_planner/set_collision_cache \
  curobo_msgs/srv/SetCollisionCache "{obb: 400, mesh: -1, blox: -1}"

# Apply changes
ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger

# Retry adding object
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject "{...}"
```

#### Issue 2: Cache Changes Not Taking Effect

**Symptoms:** After calling `set_collision_cache`, still getting cache exceeded errors

**Possible Causes:**
- Forgot to call `update_motion_gen_config`
- Called with wrong cache type

**Solution:**
```bash
# Always follow this two-step pattern
ros2 service call /unified_planner/set_collision_cache \
  curobo_msgs/srv/SetCollisionCache "{obb: 500, mesh: -1, blox: -1}"

# This second call is REQUIRED
ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger
```

#### Issue 3: GPU Out of Memory

**Symptoms:**
```
CUDA error: out of memory
RuntimeError: CUDA out of memory
```

**Possible Causes:**
- Cache sizes set too large for available GPU memory
- Too many meshes with small voxel_size

**Solutions:**
```bash
# Option 1: Reduce cache sizes
ros2 service call /unified_planner/set_collision_cache \
  curobo_msgs/srv/SetCollisionCache "{obb: 150, mesh: -1, blox: 10}"
ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger

# Option 2: Increase voxel_size (fewer cuboids per mesh)
ros2 param set /unified_planner voxel_size 0.08
ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger

# Option 3: Remove unnecessary meshes
ros2 service call /unified_planner/remove_all_objects std_srvs/srv/Trigger
```

#### Issue 4: Mesh Adds Successfully But Not Visible in Planning

**Symptoms:** Mesh added without errors, but planner doesn't avoid it

**Possible Causes:**
- Mesh file path incorrect
- Mesh scale/pose incorrect
- Collision checker not seeing the mesh cuboids

**Solution:**
```bash
# 1. Verify mesh was added
ros2 service call /unified_planner/get_obstacles std_srvs/srv/Trigger

# 2. Check voxel grid includes mesh
ros2 service call /unified_planner/get_voxel_grid std_srvs/srv/Trigger

# 3. Verify OBB cache has space for cuboids
ros2 param get /unified_planner collision_cache
```

### Complete Example: Scene with Multiple Meshes and Point Cloud

```bash
# Scenario: Robot workspace with 5 meshes + point cloud camera

# Step 1: Configure collision cache for complex scene
ros2 service call /unified_planner/set_collision_cache \
  curobo_msgs/srv/SetCollisionCache "{obb: 600, mesh: -1, blox: 50}"

ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger

# Step 2: Add mesh objects
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject \
  "{name: 'table', primitive_type: 4, mesh_file_path: '/meshes/table.stl', \
    pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"

ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject \
  "{name: 'wall', primitive_type: 4, mesh_file_path: '/meshes/wall.stl', \
    pose: {position: {x: -0.3, y: 0.0, z: 0.5}, orientation: {w: 1.0}}}"

# ... add 3 more meshes ...

# Step 3: Verify all objects added successfully
ros2 service call /unified_planner/get_obstacles std_srvs/srv/Trigger

# Step 4: Point cloud camera will use Blox cache automatically
# (configured in camera config file)

# Step 5: Plan trajectory avoiding all obstacles
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/GenerateTrajectory \
  "{goal_pose: {position: {x: 0.6, y: 0.4, z: 0.5}, orientation: {w: 1.0}}}"
```

---

## Configuration File Parameters

These are defined in the robot YAML file (e.g., `config/m1013.yml`).

### Trajectory Optimization Parameters

Located in the config file under `motion_gen`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `trajopt_tsteps` | 32 | Number of waypoints in optimized trajectory |
| `num_trajopt_seeds` | 12 | Number of random seeds for trajectory optimization |
| `num_graph_seeds` | 12 | Number of random seeds for graph search |
| `interpolation_dt` | 0.03 | Time step for trajectory interpolation (seconds) |
| `acceleration_scale` | 1.0 | Scale factor for acceleration limits |
| `minimize_jerk` | True | Minimize jerk (derivative of acceleration) for smoothness |
| `finetune_trajopt_iters` | 300 | Iterations for trajectory fine-tuning |

### Collision Checking Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `collision_checker_type` | `BLOX` | Type of collision checker (BLOX = voxel-based) |
| `collision_cache` | `{'obb': 100, 'blox': 10}` | Cache sizes for collision queries |
| `self_collision_check` | True | Enable self-collision checking |
| `maximum_trajectory_dt` | 0.25 | Maximum allowed time step in trajectory (seconds) |

### IK Parameters

Located under `ik_solver`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `num_seeds` | 20 | Number of random seeds for IK solving |
| `position_threshold` | 0.005 | Position error threshold (meters) |
| `rotation_threshold` | 0.05 | Rotation error threshold (radians) |
| `self_collision_check` | True | Check self-collision in IK solutions |

---

## Parameter Tuning Workflow

Here's how to tune parameters for your application:

### Step 1: Start with Defaults

```bash
ros2 launch curobo_ros gen_traj.launch.py
```

Test your application and identify issues.

### Step 2: Tune Speed

If motion is too fast/slow:

```bash
# Slower (safer)
ros2 param set /unified_planner time_dilation_factor 0.3

# Faster (when confident)
ros2 param set /unified_planner time_dilation_factor 0.8
```

### Step 3: Tune Collision Checking

If planning is slow or missing obstacles:

```bash
# For small obstacles (slower)
ros2 launch curobo_ros gen_traj.launch.py voxel_size:=0.02

# For large obstacles (faster)
ros2 launch curobo_ros gen_traj.launch.py voxel_size:=0.08
```

### Step 4: Tune Reliability

If planning often fails:

```bash
# More attempts
ros2 param set /unified_planner max_attempts 3

# Longer timeout
ros2 param set /unified_planner timeout 10.0

# Looser collision constraints
ros2 launch curobo_ros gen_traj.launch.py collision_activation_distance:=0.04
```

### Step 5: Measure and Iterate

```bash
# Monitor planning time
ros2 topic echo /unified_planner/planning_time

# Monitor success rate
# (keep track in your application)
```

---

## Common Parameter Combinations

### For Testing/Development

```bash
ros2 launch curobo_ros gen_traj.launch.py \
  time_dilation_factor:=0.3 \
  voxel_size:=0.05 \
  max_attempts:=2 \
  timeout:=10.0
```

**Why**: Slow motion, reliable planning, generous timeout.

### For Production (Fast)

```bash
ros2 launch curobo_ros gen_traj.launch.py \
  time_dilation_factor:=0.8 \
  voxel_size:=0.05 \
  max_attempts:=1 \
  timeout:=2.0
```

**Why**: Fast motion, quick planning, minimal overhead.

### For Precision Tasks

```bash
ros2 launch curobo_ros gen_traj.launch.py \
  time_dilation_factor:=0.4 \
  voxel_size:=0.02 \
  collision_activation_distance:=0.02 \
  max_attempts:=3
```

**Why**: Slow motion, high resolution collision checking, tight clearances.

### For Cluttered Environments

```bash
ros2 launch curobo_ros gen_traj.launch.py \
  time_dilation_factor:=0.5 \
  voxel_size:=0.03 \
  collision_activation_distance:=0.03 \
  max_attempts:=5 \
  timeout:=15.0
```

**Why**: Moderate speed, good resolution, multiple attempts, generous timeout.

---

## Summary Table

| Parameter | Type | Default | Range | Impact | Tuning Goal |
|-----------|------|---------|-------|--------|-------------|
| `voxel_size` | Launch | 0.05 m | 0.01-0.2 m | Planning speed vs accuracy | Balance speed and precision |
| `time_dilation_factor` | Runtime | 0.5 | 0.0-1.0 | Robot speed | Balance speed and safety |
| `collision_activation_distance` | Launch | 0.025 m | 0.01-0.1 m | Safety margin | Balance safety and reachability |
| `max_attempts` | Runtime | 1 | 1-10 | Success rate vs latency | Increase for difficult problems |
| `timeout` | Runtime | 5.0 s | 0.5-30.0 s | Responsiveness vs completeness | Match application needs |

---

## Debugging with Parameters

### Problem: Planning is too slow

**Try**:
```bash
# Larger voxels
ros2 launch curobo_ros gen_traj.launch.py voxel_size:=0.08

# Fewer attempts
ros2 param set /unified_planner max_attempts 1

# Shorter timeout
ros2 param set /unified_planner timeout 2.0
```

### Problem: Missing small obstacles

**Try**:
```bash
# Smaller voxels
ros2 launch curobo_ros gen_traj.launch.py voxel_size:=0.02
```

### Problem: Can't find paths in tight spaces

**Try**:
```bash
# Tighter collision threshold
ros2 launch curobo_ros gen_traj.launch.py collision_activation_distance:=0.01

# More attempts
ros2 param set /unified_planner max_attempts 5
```

### Problem: Robot moves too fast/slow

**Try**:
```bash
# Adjust speed
ros2 param set /unified_planner time_dilation_factor 0.3  # slower
ros2 param set /unified_planner time_dilation_factor 0.8  # faster
```

---

## Next Steps

- **[Your First Trajectory](../tutorials/1_first_trajectory.md)** - Practice using parameters
- **[Adding Your Robot](../tutorials/02-adding-your-robot.md)** - Configure parameters for your robot
- **[Troubleshooting](../troubleshooting.md)** - Solutions to common parameter-related issues
