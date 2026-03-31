# Tutorial 3: Managing Collision Objects

This tutorial covers everything about adding, managing, and debugging collision objects in curobo_ros. You'll learn how to:

- Add different types of obstacles (boxes, spheres, cylinders, meshes)
- Manage objects dynamically
- Visualize collision checking
- Debug collision issues
- Use objects with both motion planning and IK

---

## Overview

curobo_ros supports dynamic obstacle management. You can add or remove objects at runtime, and the planner will automatically avoid them.

**Key Features:**
- Multiple object types (cuboid, sphere, cylinder, capsule, mesh)
- Real-time updates (no restart needed)
- Works with both trajectory generation and IK
- Voxel-based collision checking (GPU-accelerated)
- Visualization in RViz

---

## Prerequisites

- unified_planner node running
- Basic understanding of 3D coordinates and orientations

```bash
# Launch the system
ros2 launch curobo_ros gen_traj.launch.py
```

---

## Object Types

curobo_ros supports 5 object types:

| Type | ID | Description | Required Dimensions |
|------|----|--------------|--------------------|
| **CUBOID** | 0 | Box with width, length, height | `x, y, z` |
| **SPHERE** | 1 | Ball with radius | `x = y = z = radius` |
| **CAPSULE** | 2 | Cylinder with hemispherical caps | `x = y = radius`, `z = length` |
| **CYLINDER** | 3 | Cylinder with radius and height | `x = y = radius`, `z = height` |
| **MESH** | 4 | Custom mesh from file | `x, y, z` = scale factors |

**Note:** Non-cuboid primitives are handled as their respective cuRobo geometry types. Meshes are voxelized into OBBs for the BLOX collision checker.

---

## Adding Objects

### 1. Adding a Box (Cuboid)

```bash
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject \
"{
  name: 'table',
  type: 0,
  pose: {
    position: {x: 0.5, y: 0.0, z: 0.2},
    orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
  },
  dimensions: {x: 1.0, y: 0.6, z: 0.05},
  color: {r: 0.7, g: 0.5, b: 0.3, a: 0.9}
}"
```

**Parameters:**
- `name`: Unique identifier (used to remove later)
- `type`: 0 for cuboid
- `pose.position`: Center of the box (meters)
- `pose.orientation`: Quaternion (for cuboids, rotation is supported)
- `dimensions`: Size in x, y, z (meters)
- `color`: RGBA (0-1 range) for visualization

**Response:**
```yaml
success: true
message: "Object 'table' added successfully (1 cuboids, 0 mesh in world)"
```

### 2. Adding a Sphere

```bash
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject \
"{
  name: 'ball',
  type: 1,
  pose: {
    position: {x: 0.6, y: 0.3, z: 0.4},
    orientation: {w: 1.0, x: 0, y: 0, z: 0}
  },
  dimensions: {x: 0.08, y: 0.08, z: 0.08},
  color: {r: 1.0, g: 0.0, b: 0.0, a: 0.8}
}"
```

**Important**: For spheres, set `x = y = z = radius`.

### 3. Adding a Cylinder

```bash
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject \
"{
  name: 'pillar',
  type: 3,
  pose: {
    position: {x: 0.4, y: -0.3, z: 0.25},
    orientation: {w: 1.0, x: 0, y: 0, z: 0}
  },
  dimensions: {x: 0.05, y: 0.05, z: 0.5},
  color: {r: 0.5, g: 0.5, b: 0.5, a: 0.9}
}"
```

**Dimensions:**
- `x = y`: Radius of cylinder
- `z`: Height of cylinder

**Note:** Cylinder is aligned with Z-axis by default.

### 4. Adding a Mesh

```bash
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject \
"{
  name: 'custom_part',
  type: 4,
  pose: {
    position: {x: 0.5, y: 0.0, z: 0.3},
    orientation: {w: 1.0, x: 0, y: 0, z: 0}
  },
  mesh_file_path: '/path/to/mesh.stl',
  dimensions: {x: 1.0, y: 1.0, z: 1.0},
  color: {r: 0.0, g: 1.0, b: 1.0, a: 0.8}
}"
```

**Parameters:**
- `mesh_file_path`: Absolute path to `.stl` or `.obj` mesh file
- `dimensions`: Scale factors for x, y, z axes (1.0 = original size, 0.01 = 1/100th scale)

**Important:** Meshes are voxelized into oriented bounding boxes (OBBs) for the BLOX collision checker. Set the OBB cache large enough before adding the mesh (see [Tutorial 3: Collision Cache](../concepts/parameters.md#collision-cache-parameters)).

---

## Managing Objects

### List All Objects

```bash
ros2 service call /unified_planner/get_obstacles std_srvs/srv/Trigger
```

**Response:**
```yaml
success: true
message: "table, ball, pillar, custom_part"
```

### Remove Specific Object

```bash
ros2 service call /unified_planner/remove_object curobo_msgs/srv/RemoveObject \
  "{name: 'ball'}"
```

**Response:**
```yaml
success: true
message: "Object 'ball' removed successfully (sphere)"
```

### Remove All Objects

```bash
ros2 service call /unified_planner/remove_all_objects std_srvs/srv/Trigger
```

**Response:**
```yaml
success: true
message: "All objects removed successfully (0 cuboids, 0 meshes)"
```

---

## Disabling Link Collision Spheres

You can enable or disable collision spheres for specific robot links at runtime.
This is useful to allow the gripper to make contact with an object during grasping,
or to ignore self-collision for a specific link during a constrained motion.

All solvers (MotionGen, MPC) share the same kinematics configuration, so a single
call immediately affects every active planner.

### Disable a link

```bash
ros2 service call /unified_planner/set_link_collision \
  curobo_msgs/srv/SetLinkCollision \
  "{link_names: ['link6'], enabled: false}"
```

**Response:**
```yaml
success: true
message: "Collision disabled for ['link6']"
applied_links: ['link6']
unknown_links: []
```

### Disable multiple links

```bash
ros2 service call /unified_planner/set_link_collision \
  curobo_msgs/srv/SetLinkCollision \
  "{link_names: ['link5', 'link6'], enabled: false}"
```

### Re-enable links

```bash
ros2 service call /unified_planner/set_link_collision \
  curobo_msgs/srv/SetLinkCollision \
  "{link_names: ['link5', 'link6'], enabled: true}"
```

**Notes:**
- The state persists until an explicit call with the opposite `enabled` value.
- Unknown link names are reported in `unknown_links` and ignored.
- Disabled spheres are hidden from the collision sphere visualization (see below).

---

## Visualizing Collision Checking

### Robot Collision Spheres

The robot is represented as a collection of spheres for collision checking.
Publishing is **disabled by default** to avoid interference during MPC initialization.

**Enable publishing:**
```bash
ros2 service call /unified_planner/set_collision_spheres_enabled \
  std_srvs/srv/SetBool "{data: true}"
```

**View collision spheres:**
```bash
ros2 topic echo /unified_planner/collision_spheres
```

**In RViz:**
1. Add → MarkerArray
2. Topic: `/unified_planner/collision_spheres`
3. You'll see red semi-transparent spheres covering the robot

Spheres belonging to disabled links (via `set_link_collision`) are automatically
hidden from this visualization.

**Update rate:** 0.5 Hz (twice per second)

### Voxel Grid Visualization

The environment is discretized into voxels for collision checking. Be carefull, this kind of operation is really GPU demanding and could ask a lot of VRAM.

**Get voxel grid:**
```bash
ros2 service call /unified_planner/get_voxel_grid curobo_msgs/srv/GetVoxelGrid
```

**Response** includes:
- Voxel grid dimensions
- Occupied voxels
- Resolution (voxel size)

---

## Collision Distance Checking

Check how close the robot is to obstacles:

```bash
ros2 service call /unified_planner/get_collision_distance curobo_msgs/srv/GetCollisionDistance
```

**Response:**
```yaml
data: [0.15, 0.23, 0.08, ...]  # Distance for each collision sphere
nb_sphere: 13  
```

**Interpretation:**
- Positive values: Distance to nearest obstacle (meters)
- Negative values: Penetration depth (collision!)
- Large values: Far from obstacles

**Use case:** Safety monitoring during execution.

---

## Example: Building a Scene

Let's build a complete scene with multiple obstacles:

```bash
# 1. Add a table
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject \
"{name: 'table', type: 0,
  pose: {position: {x: 0.5, y: 0.0, z: 0.15}, orientation: {w: 1.0, x: 0, y: 0, z: 0}},
  dimensions: {x: 0.8, y: 1.0, z: 0.05},
  color: {r: 0.6, g: 0.4, b: 0.2, a: 0.9}}"

# 2. Add walls
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject \
"{name: 'wall_left', type: 0,
  pose: {position: {x: 0.5, y: -0.6, z: 0.5}, orientation: {w: 1.0, x: 0, y: 0, z: 0}},
  dimensions: {x: 1.5, y: 0.05, z: 1.0},
  color: {r: 0.8, g: 0.8, b: 0.8, a: 0.5}}"

ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject \
"{name: 'wall_right', type: 0,
  pose: {position: {x: 0.5, y: 0.6, z: 0.5}, orientation: {w: 1.0, x: 0, y: 0, z: 0}},
  dimensions: {x: 1.5, y: 0.05, z: 1.0},
  color: {r: 0.8, g: 0.8, b: 0.8, a: 0.5}}"

# 3. Add a hanging obstacle
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject \
"{name: 'ceiling_lamp', type: 1,
  pose: {position: {x: 0.6, y: 0.2, z: 0.8}, orientation: {w: 1.0, x: 0, y: 0, z: 0}},
  dimensions: {x: 0.15, y: 0.15, z: 0.15},
  color: {r: 1.0, g: 1.0, b: 0.0, a: 0.7}}"

# 4. Add objects on table
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject \
"{name: 'box_on_table', type: 0,
  pose: {position: {x: 0.4, y: 0.1, z: 0.25}, orientation: {w: 1.0, x: 0, y: 0, z: 0}},
  dimensions: {x: 0.1, y: 0.1, z: 0.15},
  color: {r: 0.0, g: 0.5, b: 1.0, a: 0.9}}"
```

**Now plan a trajectory - it will avoid all obstacles!**

```bash
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
  "{target_pose: {position: {x: 0.6, y: 0.0, z: 0.5}, orientation: {w: 1.0, x: 0, y: 0, z: 0}}}"
```

---

## Using IK with Obstacles

The IK solver on the unified planner shares the same obstacle world as the trajectory planners. Any object added via `add_object` is automatically visible to IK — no separate setup needed.

```bash
# Add an obstacle
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject \
  "{name: 'obstacle', type: 0, dimensions: {x: 0.2, y: 0.2, z: 0.2}, \
    pose: {position: {x: 0.5, y: 0.0, z: 0.4}, orientation: {w: 1.0}}}"

# Warmup IK (required once)
ros2 service call /unified_planner/warmup_ik curobo_msgs/srv/WarmupIK "{batch_size: 1}"

# Solve IK — will return failure if target collides with the obstacle
ros2 service call /unified_planner/ik curobo_msgs/srv/Ik \
  "{pose: {position: {x: 0.5, y: 0.0, z: 0.4}, orientation: {w: 1.0}}}"
```

See [Tutorial 6: IK/FK Services](06-ik-fk-services.md) for the full IK workflow.

---

For the complete service and topic reference, see [ROS Interfaces — Collision Management](../concepts/ros-interfaces.md#collision-management-services).

---

## Troubleshooting

### Objects not appearing in RViz

**Check:**
1. Object was added successfully (check service response)
2. RViz has MarkerArray display for voxels
3. Object is within view frustum
4. Color alpha > 0 (not transparent)

### Planning ignores obstacles

**Possible causes:**
- Obstacle is outside planning space
- `voxel_size` is too large (obstacle falls in "free" voxel)
- Collision checking disabled (check config)

**Solutions:**
```bash
# Use smaller voxels
ros2 launch curobo_ros gen_traj.launch.py voxel_size:=0.03

# Reload configuration
ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger
```

### False collisions

**Cause:** Collision spheres too large or obstacle bounding box too large

**Solutions:**
- Refine robot collision spheres in config file
- Use cuboids instead of other shapes (more accurate bounding boxes)
- Reduce `collision_activation_distance`

---

## Summary

**You've learned:**

- ✅ Adding different object types (cuboid, sphere, cylinder, mesh)
- ✅ Managing objects dynamically
- ✅ Disabling collision spheres per link (`set_link_collision`)
- ✅ Visualizing collision checking and disabled links
- ✅ Debugging collision issues

---

## Next Steps

- **[Robot Execution](04-robot-execution.md)** - Connect to real robot or emulator
- **[Camera Integration](07-pointcloud-detection.md)** - Detect obstacles automatically with cameras
- **[Parameters Guide](../concepts/parameters.md)** - Tune collision checking parameters

