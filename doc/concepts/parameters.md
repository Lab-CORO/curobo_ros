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
ros2 param set /curobo_gen_traj voxel_size 0.02
ros2 service call /curobo_gen_traj/update_motion_gen_config std_srvs/srv/Trigger
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
ros2 param set /curobo_gen_traj time_dilation_factor 0.7
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
ros2 param set /curobo_gen_traj max_attempts 3
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
ros2 param set /curobo_gen_traj timeout 2.0
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

See [Adding Your Robot Tutorial](../tutorials/2_adding_your_robot.md) for details.

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
ros2 param set /curobo_gen_traj time_dilation_factor 0.3

# Faster (when confident)
ros2 param set /curobo_gen_traj time_dilation_factor 0.8
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
ros2 param set /curobo_gen_traj max_attempts 3

# Longer timeout
ros2 param set /curobo_gen_traj timeout 10.0

# Looser collision constraints
ros2 launch curobo_ros gen_traj.launch.py collision_activation_distance:=0.04
```

### Step 5: Measure and Iterate

```bash
# Monitor planning time
ros2 topic echo /curobo_gen_traj/planning_time

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
ros2 param set /curobo_gen_traj max_attempts 1

# Shorter timeout
ros2 param set /curobo_gen_traj timeout 2.0
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
ros2 param set /curobo_gen_traj max_attempts 5
```

### Problem: Robot moves too fast/slow

**Try**:
```bash
# Adjust speed
ros2 param set /curobo_gen_traj time_dilation_factor 0.3  # slower
ros2 param set /curobo_gen_traj time_dilation_factor 0.8  # faster
```

---

## Next Steps

- **[Your First Trajectory](../tutorials/1_first_trajectory.md)** - Practice using parameters
- **[Adding Your Robot](../tutorials/2_adding_your_robot.md)** - Configure parameters for your robot
- **[Troubleshooting](../troubleshooting.md)** - Solutions to common parameter-related issues
