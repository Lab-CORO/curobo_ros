# Tutorial 3: Managing Collision Objects

**Difficulty**: Beginner · **Time**: ~25 min · **Prerequisites**: [Tutorial 1](01-first-trajectory.md)

Everything about building the collision world: adding primitives and meshes, attaching an object to the arm, toggling per-link collision, and debugging what the planner actually sees. All obstacles are shared by every solver (planning, MPC, IK) — add once, avoided everywhere.

## Object types and dimensions

`add_object` takes a `type`, a `pose` (robot base frame), and a `dimensions` vector whose meaning depends on the type:

| Type | ID | `dimensions.x` | `dimensions.y` | `dimensions.z` |
|---|---|---|---|---|
| Cuboid | 0 | full extent X | full extent Y | full extent Z |
| Sphere | 1 | radius | — | — |
| Capsule | 2 | radius | — | length |
| Cylinder | 3 | radius | — | height |
| Mesh | 4 | scale X | scale Y | scale Z |

## Adding primitives

```bash
# A tabletop: 0.8 x 1.0 x 0.05 m
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject \
  "{type: 0, name: 'table', pose: {position: {x: 0.6, y: 0.0, z: 0.1}, orientation: {w: 1.0}}, \
    dimensions: {x: 0.8, y: 1.0, z: 0.05}, color: {r: 0.5, g: 0.3, b: 0.1, a: 1.0}}"

# A ball of radius 10 cm
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject \
  "{type: 1, name: 'ball', pose: {position: {x: 0.4, y: -0.3, z: 0.5}, orientation: {w: 1.0}}, \
    dimensions: {x: 0.1, y: 0.1, z: 0.1}, color: {r: 1.0, g: 0.0, b: 0.0, a: 1.0}}"

# A pole: cylinder radius 5 cm, height 1 m
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject \
  "{type: 3, name: 'pole', pose: {position: {x: 0.7, y: 0.4, z: 0.5}, orientation: {w: 1.0}}, \
    dimensions: {x: 0.05, y: 0.05, z: 1.0}, color: {r: 0.2, g: 0.2, b: 0.8, a: 1.0}}"
```

Names must be unique; adding a duplicate name fails. Obstacles appear in RViz through the `/unified_planner/scene_obstacles` markers (refreshed every 2 s).

## Adding a mesh

```bash
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject \
  "{type: 4, name: 'duck', mesh_file_path: '/absolute/path/to/duck.stl', \
    pose: {position: {x: 0.5, y: 0.3, z: 0.2}, orientation: {w: 1.0}}, \
    dimensions: {x: 1.0, y: 1.0, z: 1.0}, color: {g: 0.8, a: 1.0}}"
```

`mesh_file_path` must be an **absolute path** (`.stl` or `.obj`), readable from inside the container. `dimensions` acts as a scale factor. In cuRobo v2 a mesh goes directly into the `Scene` — no voxelization step on your side.

## Listing and removing

```bash
ros2 service call /unified_planner/get_obstacles std_srvs/srv/Trigger        # names, one per line
ros2 service call /unified_planner/remove_object curobo_msgs/srv/RemoveObject "{name: 'ball'}"
ros2 service call /unified_planner/remove_all_objects std_srvs/srv/Trigger
```

## Attaching an object to the robot

For pick-and-place: attach a scene object to the flange so it moves with the arm and is collision-checked as part of the robot.

```bash
# The object must already exist in the scene
ros2 service call /unified_planner/attach_object curobo_msgs/srv/AttachObject "{object_name: 'ball'}"
# ... move around ...
ros2 service call /unified_planner/detach_object std_srvs/srv/Trigger
```

This requires the robot configuration to define an attachable link — the shipped M1013 config (`curobo_doosan/src/m1013/m1013.yml`) already does:

```yaml
kinematics:
  extra_links:
    attached_object:          # fixed joint on the flange
      parent_link_name: dsr01/link6
      ...
  collision_link_names: [..., attached_object]
  extra_collision_spheres: {attached_object: 4}
```

If you integrated your own robot ([Tutorial 2](02-adding-your-robot.md)), replicate this block to enable attachment.

## Per-link collision toggling

When the gripper must contact the part it grasps, disable collision for those links instead of removing the object:

```bash
ros2 service call /unified_planner/set_link_collision curobo_msgs/srv/SetLinkCollision \
  "{link_names: ['dsr01/link6'], enabled: false}"
```

The response reports `applied_links` / `unknown_links`. The state applies to **all** solvers and persists until the opposite call re-enables it.

## Seeing what the planner sees

Three debugging tools, in increasing order of detail:

```bash
# 1. Robot collision spheres as markers (off by default)
ros2 service call /unified_planner/set_collision_spheres_enabled std_srvs/srv/SetBool "{data: true}"

# 2. Distance of every robot sphere to the nearest obstacle, at the current state
ros2 service call /unified_planner/get_collision_distance curobo_msgs/srv/GetCollisionDistance

# 3. The collision world rasterized as voxels, in a bounding box you choose
ros2 service call /unified_planner/get_voxel_grid curobo_msgs/srv/GetVoxelGrid \
  "{bbox_min_x: -1.0, bbox_min_y: -1.0, bbox_min_z: -0.2, bbox_max_x: 1.0, bbox_max_y: 1.0, bbox_max_z: 1.2}"
```

If a plan surprises you, trust these over the RViz markers — they read the actual collision world.

## Obstacles and IK

Collision-aware IK shares the same world. With the pole from above still in the scene:

```bash
ros2 service call /unified_planner/warmup_ik curobo_msgs/srv/WarmupIK "{batch_size: 1}"
ros2 service call /unified_planner/ik curobo_msgs/srv/Ik \
  "{pose: {position: {x: 0.7, y: 0.4, z: 0.5}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}"
```

A pose *inside* the pole returns no valid solution. See [Tutorial 6](06-ik-fk-services.md).

## Capacity: the collision caches

The solvers pre-allocate space for obstacles. If `add_object` starts failing with a cache error, grow the cache (this rebuilds the solvers, ~20 s, and is refused during execution):

```bash
ros2 service call /unified_planner/set_collision_cache curobo_msgs/srv/SetCollisionCache \
  "{obb: 200, mesh: -1, blox: -1}"
```

Details in the [Parameters Guide](../concepts/parameters.md) and [ROS Interfaces](../concepts/ros-interfaces.md).

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Object added but planner ignores it | Check `get_obstacles`; if listed, verify pose/dimensions units (metres, full extents for cuboids) |
| `add_object` fails on a mesh | Path not absolute, or not readable inside the container |
| Attach fails | Object name not in the scene, or robot config lacks `attached_object` |
| Adding objects fails after many adds | Collision cache full — grow it (see above) |

## Next steps

- [Tutorial 7: Camera-Based Obstacle Detection](07-pointcloud-detection.md) — automatic obstacles from depth cameras
- [Tutorial 4: Robot Execution](04-robot-execution.md)

[← Tutorial 2](02-adding-your-robot.md) | [Tutorial 4 →](04-robot-execution.md)
