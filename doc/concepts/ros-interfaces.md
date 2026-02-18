# ROS 2 Interfaces - Complete API Reference

Complete reference for all topics, services, actions, and parameters in curobo_ros.

## Table of Contents

- [Topics](#topics)
- [Services](#services)
  - [Motion Planning](#motion-planning-services)
  - [Collision Management](#collision-management-services)
  - [Kinematics](#kinematics-services)
  - [Configuration](#configuration-services)
- [Actions](#actions)
- [Parameters](#parameters)

---

## Topics

### Publishers

| Topic Name | Message Type | Node | Description | Frequency |
|------------|--------------|------|-------------|-----------|
| `<node_name>/collision_spheres` | `visualization_msgs/MarkerArray` | unified_planner | Robot collision spheres visualization | On update |

### Subscribers

| Topic Name | Message Type | Node | Description |
|------------|--------------|------|-------------|
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | unified_planner | Depth camera point cloud for obstacle detection |
| `/joint_states` | `sensor_msgs/JointState` | unified_planner | Current robot joint positions |

---

## Services

### Motion Planning Services

#### 1. `generate_trajectory` - Generate Motion Trajectory (UPDATED Dec 5, 2025)

**Service Type**: `curobo_msgs/srv/TrajectoryGeneration`

**Purpose**: Generate collision-free motion trajectory from start to target pose(s).

**Request Fields:**
- `start_pose` (JointState): Starting joint configuration
- `target_pose` (Pose): Single target end-effector pose
- `target_poses` (Pose[]): **NEW** Multiple waypoints for multi-point planning
- `target_joint_positions` (float64[]): **NEW** Target in joint space
- `trajectory_constraints` (int8[]): **NEW** Constraints [theta_x, theta_y, theta_z, x, y, z]
- `trajectories_contraints` (int8[]): **NEW** Per-waypoint constraints (flattened)

**Response Fields:**
- `success` (bool): True if trajectory generated successfully
- `message` (string): Status message
- `trajectory` (JointState[]): **NEW (Dec 5)** Full trajectory returned in response
- `dt` (float64): **NEW (Dec 5)** Time step between waypoints (seconds)

**What's New (Dec 5, 2025)**:
- Trajectory now returned directly in response
- Support for multi-point waypoint planning
- Support for joint-space target planning
- Support for trajectory constraints (rotation, translation)

**ROS2 CLI Examples:**

```bash
# 1. Single-point Cartesian planning (classic)
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
  "{target_pose: {position: {x: 0.5, y: 0.3, z: 0.4}, orientation: {w: 1.0}}}"

# 2. Multi-point waypoint planning (NEW)
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
  "{target_poses: [
    {position: {x: 0.5, y: 0.2, z: 0.4}, orientation: {w: 1.0}},
    {position: {x: 0.6, y: 0.2, z: 0.4}, orientation: {w: 1.0}},
    {position: {x: 0.7, y: 0.2, z: 0.4}, orientation: {w: 1.0}}
  ]}"

# 3. Joint-space planning (NEW)
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
  "{target_joint_positions: [0.0, -0.5, 0.5, 0.0, 1.57, 0.0]}"

# 4. With trajectory constraints (NEW) - Constrain rotation around X
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
  "{target_pose: {position: {x: 0.5, y: 0.3, z: 0.4}, orientation: {w: 1.0}}, \
    trajectory_constraints: [1, 0, 0, 0, 0, 0]}"
```

**Expected Response:**
```yaml
success: true
message: 'Trajectory generated successfully'
trajectory:
  - position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    velocity: []
  - position: [0.123, -0.234, 0.345, -0.123, 0.456, 0.234]
    velocity: []
  # ... more waypoints
dt: 0.05  # 50ms between waypoints
```

**Use Cases:**
- Plan collision-free motion from current pose to target
- Multi-waypoint tasks (pick-and-place, inspection paths)
- Joint-space motions for specific joint configurations
- Constrained motions (keep orientation, move in straight line)

---

#### 2. `set_planner` - Switch Planner Type (NEW Nov 9, 2025)

**Service Type**: `curobo_msgs/srv/SetPlanner`

**Purpose**: Dynamically switch between different planning algorithms at runtime.

**Request Fields:**
- `planner_type` (uint8): Planner type constant

**Planner Type Constants:**
| Constant | Value | Planner Type | Description |
|----------|-------|--------------|-------------|
| CLASSIC | 0 | Classic motion planning | Open-loop, single-shot planning for static environments |
| MPC | 1 | Model Predictive Control | Closed-loop, reactive planning for dynamic obstacles |
| MULTIPOINT | 4 | Multi-point planning | Waypoint-based planning (includes constrained planning) |
| JOINT_SPACE | 5 | Joint-space planning | Plan in joint space (not Cartesian) |

**Note**: BATCH (2) and CONSTRAINED (3) planner types are not implemented. Constrained planning is included in MULTIPOINT and CLASSIC planners.

**Response Fields:**
- `success` (bool): True if planner switched successfully
- `message` (string): Status message
- `previous_planner` (string): Name of previous planner
- `current_planner` (string): Name of active planner

**ROS2 CLI Examples:**

```bash
# Switch to MPC planner
ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 1}"

# Switch to classic planner
ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 0}"

# Switch to multi-point planner
ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 4}"
```

**Expected Response:**
```yaml
success: true
message: 'Planner switched successfully'
previous_planner: 'ClassicPlanner'
current_planner: 'MPCPlanner'
```

**Use Cases:**
- Switch to MPC for dynamic environments with moving obstacles
- Use MULTIPOINT for tasks requiring specific waypoints
- Use JOINT_SPACE for direct joint control

---

### Collision Management Services

#### 3. `add_object` - Add Collision Object

**Service Type**: `curobo_msgs/srv/AddObject`

**Purpose**: Add obstacle to the collision scene.

**Request Fields:**
- `name` (string): Unique object identifier
- `primitive_type` (uint8): 0=CUBOID, 1=CYLINDER, 2=SPHERE, 3=CAPSULE, 4=MESH
- `dimensions` (float64[]): Object dimensions ([x,y,z] for cuboid, [radius, height] for cylinder, etc.)
- `pose` (Pose): Object pose in base frame
- `mesh_file_path` (string): Path to mesh file (for MESH type only)

**Response Fields:**
- `success` (bool): True if object added
- `message` (string): Status message

**ROS2 CLI Examples:**

```bash
# Add cuboid (table)
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject \
  "{name: 'table', primitive_type: 0, dimensions: [1.0, 0.8, 0.05], \
    pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"

# Add sphere (ball)
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject \
  "{name: 'ball', primitive_type: 2, dimensions: [0.1], \
    pose: {position: {x: 0.4, y: 0.2, z: 0.3}, orientation: {w: 1.0}}}"

# Add cylinder (pole)
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject \
  "{name: 'pole', primitive_type: 1, dimensions: [0.05, 1.0], \
    pose: {position: {x: 0.3, y: 0.3, z: 0.5}, orientation: {w: 1.0}}}"
```

**Important Note - Mesh Handling:**

When adding a MESH object, curobo internally converts it into multiple cuboids that are fused together to reduce the total number. This approach optimizes collision checking performance.

**Implications:**
- **Collision cache**: You need to increase the **cuboid (OBB) collision cache** size when adding mesh objects, not the mesh cache
- **Naming convention**: Each cuboid generated from a mesh will be named `{mesh_name}_cuboid_{id}`
  - Example: A mesh named "complex_part" will create cuboids named "complex_part_cuboid_0", "complex_part_cuboid_1", etc.
- **Cache sizing**: For a complex mesh that generates 20 cuboids, ensure your OBB cache size is sufficient

**Example with mesh:**
```bash
# Add mesh object (will be converted to cuboids internally)
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject \
  "{name: 'part', primitive_type: 4, mesh_file_path: '/path/to/mesh.stl', \
    pose: {position: {x: 0.5, y: 0.0, z: 0.3}, orientation: {w: 1.0}}}"

# Increase OBB cache to accommodate the generated cuboids
ros2 service call /unified_planner/set_collision_cache \
  curobo_msgs/srv/SetCollisionCache "{obb: 200, mesh: -1, blox: -1}"

# Apply the cache changes
ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger
```

---

#### 4. `remove_object` - Remove Collision Object

**Service Type**: `curobo_msgs/srv/RemoveObject`

**Purpose**: Remove specific obstacle from scene.

**Request Fields:**
- `name` (string): Object name to remove

**Response Fields:**
- `success` (bool): True if removed
- `message` (string): Status message

**ROS2 CLI Example:**

```bash
ros2 service call /unified_planner/remove_object curobo_msgs/srv/RemoveObject "{name: 'table'}"
```

---

#### 5. `get_obstacles` - List All Obstacles

**Service Type**: `std_srvs/srv/Trigger`

**Purpose**: Retrieve list of all current obstacles in the scene.

**ROS2 CLI Example:**

```bash
ros2 service call /unified_planner/get_obstacles std_srvs/srv/Trigger
```

**Expected Response:**
```yaml
success: true
message: 'Obstacles: table, ball, pole'
```

---

#### 6. `remove_all_objects` - Clear All Obstacles

**Service Type**: `std_srvs/srv/Trigger`

**Purpose**: Remove all obstacles from the scene.

**ROS2 CLI Example:**

```bash
ros2 service call /unified_planner/remove_all_objects std_srvs/srv/Trigger
```

---

#### 7. `get_voxel_grid` - Get Voxelized Environment

**Service Type**: `curobo_msgs/srv/GetVoxelGrid`

**Purpose**: Retrieve voxelized representation of current collision scene.

**Response Fields:**
- `success` (bool)
- `message` (string)
- `voxel_grid` (PointCloud2): Occupied voxels as point cloud

**ROS2 CLI Example:**

```bash
ros2 service call /unified_planner/get_voxel_grid curobo_msgs/srv/GetVoxelGrid
```

**Important Note:**
The voxel area is currently a **hardcoded cube of 1.52m** centered on the robot base. Obstacles outside this region will not appear in the voxel grid.

**Use Cases:**
- Visualize collision voxels in RViz (within 1.52m cube)
- Debug collision checking
- Export scene representation

---

#### 8. `get_collision_distance` - Query Collision Distances

**Service Type**: `curobo_msgs/srv/GetCollisionDistance`

**Purpose**: Compute distances to nearest obstacles for a given robot configuration.

**Request Fields:**
- `joint_state` (JointState): Robot joint configuration

**Response Fields:**
- `success` (bool)
- `message` (string)
- `distances` (float64[]): Distance per collision sphere (meters)
  - **Positive value**: Distance to nearest obstacle (meters)
  - **Negative value**: Sphere is in collision (penetration depth)
- `in_collision` (bool): True if any collision detected

**ROS2 CLI Example:**

```bash
ros2 service call /unified_planner/get_collision_distance curobo_msgs/srv/GetCollisionDistance \
  "{joint_state: {position: [0.0, -0.5, 0.5, 0.0, 1.57, 0.0]}}"
```

**Expected Response:**
```yaml
success: true
message: 'Collision distances computed'
distances: [0.05, 0.12, -0.02, 0.15, 0.25, 0.30, 0.18]  # Third sphere in collision (-0.02m penetration)
in_collision: true
```

**Interpretation:**
- `distances: [0.05, 0.12, -0.02, ...]`: Third sphere has -0.02m distance (in collision)
- `in_collision: true`: At least one sphere is in collision

---

#### 9. `set_collision_cache` - Configure Collision Cache (NEW Dec 12, 2025)

**Service Type**: `curobo_msgs/srv/SetCollisionCache`

**Purpose**: Dynamically configure collision cache sizes to optimize GPU memory usage.

**Request Fields:**
- `obb` (int32): OBB cache size (-1 = don't modify)
- `mesh` (int32): Mesh cache size (-1 = don't modify)
- `blox` (int32): Voxel cache size (-1 = don't modify)

**Response Fields:**
- `success` (bool): True if cache updated
- `message` (string): Status message
- `obb_cache` (int32): Current OBB cache size
- `mesh_cache` (int32): Current mesh cache size
- `blox_cache` (int32): Current voxel cache size

**IMPORTANT**: After modifying collision cache parameters, you **must call** `update_motion_gen_config` service to apply the changes.

**ROS2 CLI Examples:**

```bash
# Increase mesh cache for complex scenes
ros2 service call /unified_planner/set_collision_cache \
  curobo_msgs/srv/SetCollisionCache "{obb: -1, mesh: 50, blox: -1}"

# REQUIRED: Apply the changes
ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger

# Query current cache sizes (without modifying)
ros2 service call /unified_planner/set_collision_cache \
  curobo_msgs/srv/SetCollisionCache "{obb: -1, mesh: -1, blox: -1}"
```

**Expected Response:**
```yaml
success: true
message: 'Collision cache updated successfully'
obb_cache: 100
mesh_cache: 50
blox_cache: 10
```

**Use Cases:**
- Optimize memory for scenes with many mesh objects (increase OBB cache)
- Adjust cache for point cloud environments (increase blox cache)
- Balance memory usage vs performance

**See Also**: [Parameters Guide - Collision Cache](parameters.md#collision-cache-parameters)

---

### Kinematics Services

#### 10. `fk_compute` - Forward Kinematics

**Service Type**: `curobo_msgs/srv/Fk`

**Purpose**: Compute end-effector pose from joint positions.

**Request Fields:**
- `joint_state` (JointState): Joint positions

**Response Fields:**
- `success` (bool)
- `message` (string)
- `end_effector_pose` (Pose): Computed end-effector pose

**ROS2 CLI Example:**

```bash
ros2 service call /unified_planner/fk_compute curobo_msgs/srv/Fk \
  "{joint_state: {position: [0.0, -0.5, 0.5, 0.0, 1.57, 0.0]}}"
```

**See Also**: [Tutorial 6: IK/FK Services](../tutorials/06-ik-fk-services.md)

---

#### 11. `ik_batch_poses` - Inverse Kinematics

**Service Type**: `curobo_msgs/srv/Ik`

**Purpose**: Compute joint configuration for target end-effector pose.

**Request Fields:**
- `pose` (Pose): Target end-effector pose

**Response Fields:**
- `success` (bool): True if solution found
- `joint_states` (JointState): IK solution
- `joint_states_valid` (Bool): Validity flag
- `error_msg` (String): Error message if failed

**ROS2 CLI Example:**

```bash
# Single pose IK
ros2 service call /unified_planner/ik_batch_poses curobo_msgs/srv/Ik \
  "{pose: {position: {x: 0.5, y: 0.3, z: 0.4}, orientation: {w: 1.0}}}"
```

**See Also**: [Tutorial 6: IK/FK Services](../tutorials/06-ik-fk-services.md)

---

### Configuration Services

#### 12. `update_motion_gen_config` - Reload Configuration

**Service Type**: `std_srvs/srv/Trigger`

**Purpose**: Reload motion generation configuration to apply parameter changes.

**When to Use:**
This service **must be called** after modifying:
- `voxel_size` parameter
- `collision_activation_distance` parameter
- Collision cache sizes (via `set_collision_cache` service)

**ROS2 CLI Example:**

```bash
# Modify parameter
ros2 param set /unified_planner voxel_size 0.08

# REQUIRED: Apply the change
ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger
```

**Use Cases:**
- Apply parameter changes without restarting node
- Reload collision cache configuration
- Update voxel size or collision distance settings

---

### Internal/Advanced Services

#### 13. `scene_generator`, `generate_rm`, `collisions`

These services are for internal testing and advanced use cases. See source code for details.

---

## Actions

### 1. `send_trajectory` - Execute Planned Trajectory

**Action Type**: `curobo_msgs/action/SendTrajectory`

**Purpose**: Execute a planned trajectory with real-time feedback.

**Goal Fields:**
- `trajectory` (JointTrajectory): Trajectory to execute

**Feedback Fields:**
- `current_joint_state` (JointState): Current joint positions
- `progress` (float32): Execution progress [0.0 - 1.0]

**Result Fields:**
- `success` (bool): True if execution completed
- `message` (string): Status message

**ROS2 CLI Example:**

```bash
# Send trajectory action goal
ros2 action send_goal /unified_planner/send_trajectory curobo_msgs/action/SendTrajectory \
  "{trajectory: {...}}" \
  --feedback
```

**Use Case**: Execute trajectory generated by `generate_trajectory` service.

---

### 2. `mpc_move` - MPC Closed-Loop Control (NEW Nov 9, 2025)

**Action Type**: `curobo_msgs/action/MpcMove`

**Purpose**: Closed-loop MPC control with continuous replanning to reach target pose.

**Goal Fields:**
- `target_pose` (Pose): Desired end-effector pose

**Feedback Fields:**
- `joint_command` (JointState): Current joint command being executed
- `step_progression` (float32): Progress toward goal [0.0 - 1.0]

**Result Fields:**
- `success` (bool): True if target reached
- `message` (string): Status message

**Behavior:**
- Continuously replans at configurable frequency (default: 100 Hz)
- Automatically avoids dynamic obstacles
- Converges when within threshold (default: 0.01m)
- Can be canceled mid-execution

**ROS2 CLI Examples:**

```bash
# Send MPC move goal with feedback
ros2 action send_goal /unified_planner/mpc_move curobo_msgs/action/MpcMove \
  "{target_pose: {position: {x: 0.5, y: 0.3, z: 0.4}, orientation: {w: 1.0}}}" \
  --feedback

# Cancel active MPC move
ros2 action cancel_goal /unified_planner/mpc_move <goal_id>

# Check MPC action status
ros2 action list
ros2 action info /unified_planner/mpc_move
```

**Feedback Output:**
```
Feedback:
  step_progression: 0.15
Feedback:
  step_progression: 0.32
Feedback:
  step_progression: 0.58
...
Result:
  success: true
  message: 'Target reached successfully'
```

**Use Cases:**
- Reactive control in dynamic environments
- Tracking moving targets
- Real-time obstacle avoidance
- Tasks requiring continuous replanning

**See Also**: [Tutorial 5: MPC Planner](../tutorials/05-mpc-planner.md#using-mpcmove-action)

---

## Parameters

### Core Motion Planning Parameters

| Parameter | Type | Default | Description | Runtime Modifiable |
|-----------|------|---------|-------------|--------------------|
| `voxel_size` | float | 0.05 | Collision voxel size (meters) | **Yes (requires update_motion_gen_config)** |
| `time_dilation_factor` | float | 0.5 | Trajectory speed multiplier [0.0-1.0] | Yes (via rosparam) |
| `max_attempts` | int | 1 | Planning retry attempts | Yes (via rosparam) |
| `timeout` | float | 5.0 | Planning timeout (seconds) | Yes (via rosparam) |
| `collision_activation_distance` | float | 0.025 | Collision threshold (meters) | **Yes (requires update_motion_gen_config)** |

**Example - Modify voxel_size:**
```bash
# Set new voxel size
ros2 param set /unified_planner voxel_size 0.08

# REQUIRED: Apply the change
ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger
```

**Example - Modify collision_activation_distance:**
```bash
# Set new collision distance
ros2 param set /unified_planner collision_activation_distance 0.03

# REQUIRED: Apply the change
ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger
```

### Robot Configuration Parameters

| Parameter | Type | Default | Description | Runtime Modifiable |
|-----------|------|---------|-------------|--------------------|
| `robot_config_file` | string | m1013.yml | Robot configuration YAML path | No (launch only) |
| `base_link` | string | base_0 | Robot base frame name | No (launch only) |
| `world_file` | string | '' | Initial world obstacles YAML | No (launch only) |

### Collision Cache Parameters (NEW Dec 12, 2025)

| Parameter | Type | Default | Description | Runtime Modifiable |
|-----------|------|---------|-------------|--------------------|
| `obb_cache_size` | int | 100 | OBB cache size | **Yes (via service or rosparam + update_motion_gen_config)** |
| `mesh_cache_size` | int | 10 | Mesh cache size | **Yes (via service or rosparam + update_motion_gen_config)** |
| `blox_cache_size` | int | 10 | Voxel cache size | **Yes (via service or rosparam + update_motion_gen_config)** |

**Important**: Collision cache parameters can be modified via:
1. **`set_collision_cache` service** (recommended)
2. **ROS parameters** (via ros2 param set)

**Both methods require calling `update_motion_gen_config` service to apply changes.**

**Example - Via service:**
```bash
# Modify via service
ros2 service call /unified_planner/set_collision_cache \
  curobo_msgs/srv/SetCollisionCache "{obb: 200, mesh: -1, blox: -1}"

# REQUIRED: Apply the changes
ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger
```

**Example - Via ROS parameters:**
```bash
# Modify via parameter
ros2 param set /unified_planner obb_cache_size 200

# REQUIRED: Apply the change
ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger
```

### MPC Parameters (Nov-Dec 2025)

| Parameter | Type | Default | Description | Runtime Modifiable |
|-----------|------|---------|-------------|--------------------|
| `mpc_convergence_threshold` | float | 0.01 | MPC goal threshold (meters) | Yes |
| `mpc_max_iterations` | int | 1000 | Max MPC iterations | Yes |
| `mpc_horizon` | int | 10 | MPC planning horizon | Yes |
| `mpc_control_frequency` | int | 100 | MPC replan rate (Hz) | Yes |

**Example - Modify MPC parameters:**
```bash
ros2 param set /unified_planner mpc_control_frequency 150
ros2 param set /unified_planner mpc_convergence_threshold 0.005
```

### Camera Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `cameras_config_file` | string | '' | Camera configuration YAML |
| `camera_topic` | string | /camera/depth/points | Point cloud topic |
| `downsample_voxel_size` | float | 0.01 | Point cloud filter size |

---

## Service Call Patterns

### Pattern 1: Plan and Execute

```bash
# 1. Generate trajectory
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
  "{target_pose: {position: {x: 0.5, y: 0.3, z: 0.4}, orientation: {w: 1.0}}}"

# 2. Execute trajectory (if success)
ros2 action send_goal /unified_planner/send_trajectory curobo_msgs/action/SendTrajectory \
  "{trajectory: {...}}"
```

### Pattern 2: Dynamic Environment Workflow

```bash
# 1. Switch to MPC planner
ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 1}"

# 2. Use MPC action for reactive control
ros2 action send_goal /unified_planner/mpc_move curobo_msgs/action/MpcMove \
  "{target_pose: {position: {x: 0.5, y: 0.3, z: 0.4}, orientation: {w: 1.0}}}" \
  --feedback
```

### Pattern 3: Collision Scene Setup

```bash
# 1. Clear existing obstacles
ros2 service call /unified_planner/remove_all_objects std_srvs/srv/Trigger

# 2. Add new obstacles
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject "{name: 'table', ...}"
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject "{name: 'wall', ...}"

# 3. Verify obstacles
ros2 service call /unified_planner/get_obstacles std_srvs/srv/Trigger

# 4. Plan trajectory (will avoid obstacles)
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration "{...}"
```

### Pattern 4: Parameter Update Workflow

```bash
# 1. Modify parameters
ros2 param set /unified_planner voxel_size 0.08
ros2 service call /unified_planner/set_collision_cache \
  curobo_msgs/srv/SetCollisionCache "{obb: 200, mesh: -1, blox: -1}"

# 2. REQUIRED: Apply all changes
ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger

# 3. Verify changes applied
ros2 param get /unified_planner voxel_size
```

---

## Version History

### December 15, 2025
- Removed collision checker type switch

### December 12, 2025
- Added `set_collision_cache` service for dynamic cache configuration

### December 5, 2025
- Updated `generate_trajectory` service to return trajectory in response
- Added trajectory constraints support
- Added joint-space planning support

### November 27, 2025
- Added multi-point waypoint support to `generate_trajectory`

### November 9, 2025
- Added `set_planner` service for runtime planner switching
- Added `mpc_move` action for closed-loop MPC control

---

## Related Documentation

- [Tutorial 1: Your First Trajectory](../tutorials/01-first-trajectory.md) - Basic trajectory generation
- [Tutorial 3: Collision Objects](../tutorials/03-collision-objects.md) - Obstacle management
- [Tutorial 5: MPC Planner](../tutorials/05-mpc-planner.md) - MPC usage
- [Tutorial 6: IK/FK Services](../tutorials/06-ik-fk-services.md) - Kinematics
- [Parameters Guide](parameters.md) - Detailed parameter tuning
- [curobo_msgs Package](https://github.com/Lab-CORO/curobo_msgs) - Message definitions

---

[← Back to Concepts](README.md)
