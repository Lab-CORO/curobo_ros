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

For pick-and-place: attach a payload to the flange so it moves with the arm and is collision-checked as part of the robot. Unlike `add_object`, `attach_object` never touches the `Scene` — the payload's geometry travels **inline, in the request itself**. There is no preceding `add_object` step, no obstacle to remove afterward, and only one payload at a time (a second `attach_object` before `detach_object` is refused).

```bash
# The payload's geometry, in the pince (gripper), not the scene:
ros2 service call /unified_planner/attach_object curobo_msgs/srv/AttachObject \
  "{header: {frame_id: 'dsr01/link6'}, type: 0, \
    pose: {position: {x: 0.0, y: 0.0, z: 0.05}, orientation: {w: 1.0}}, \
    dimensions: {x: 0.04, y: 0.04, z: 0.04}}"
# ... move around ...
ros2 service call /unified_planner/detach_object std_srvs/srv/Trigger
```

`detach_object` takes no argument and resets the payload's collision spheres — it does **not** re-add anything to the scene. If you want the payload to reappear as an obstacle at the drop location, call `add_object` yourself after detaching; an automatic re-add was considered and rejected, because it would place the obstacle right between the gripper's fingers at the moment of release, and the very next plan would start "in collision".

### `header.frame_id`

`pose` is expressed in whichever TF frame `header.frame_id` names — this is the field most likely to be misused, and the mistake is silent:

- **The usual case: the gripper link** (`dsr01/link6` on the M1013, i.e. `tool_frames[0]` in the robot YAML). The pose is then a fixed property of the grasp — "5 cm in front of the flange" — independent of the arm's current joint configuration, exactly like the example above.
- **`frame_id: ""`** means the planner's own base frame (the same one `add_object` poses are in, e.g. `dsr01/world`) — the one case that needs no TF lookup at all. Use it when a vision pipeline already reports the part's pose there.
- **Any other TF frame** works too — including a camera's optical frame, so a vision detection can be passed through unchanged, without the caller doing its own TF composition first.

Two pitfalls, both silent:
- `attached_object` itself is **not** a TF frame — it is a virtual `extra_links` entry that only exists inside cuRobo's kinematics. Use its parent link (`dsr01/link6`) instead; naming `attached_object` here makes the TF lookup fail.
- A pose coming from a detection must be **fresh at the moment of the call** — the part has already moved once the gripper closes on it, and a stale pose by even a few centimetres shifts the whole collision model with no error raised.

`header.stamp` is **ignored**: the transform used is always the latest available, to stay consistent with the joint configuration the attachment is computed from (the arm's current position, at call time).

Attach applies to **every planner that currently has a live solver** — Classic/Multi Point/Joint Space (the shared `MotionPlanner`), MPC, and LBFGS — not just whichever one is active when you call it. The spheres are fitted once and the identical model is written into each; switching planners (`set_planner`) or triggering a solver rebuild (`set_collision_cache`) afterward carries the attachment along automatically, with no need to call `attach_object` again. **Motion Retargeting (`retarget`) is not covered**: a payload attached while some other planner is active is invisible to the retargeter, and the `ik`/`fk` services are collision-aware of everything *except* an attached payload (they use their own separate kinematics). `attach_object`/`detach_object` are refused with `success: false` while an execution goal is active — cancel it first.

### The perception side: the payload masks itself

`attach_object` only affects the **analytic** collision model (the fitted spheres). If a camera is watching the part being grasped, the depth-based perception layer (the ESDF voxel grid) would still see it and treat it as a world obstacle — the attached spheres colliding with their own voxels.

That is handled for you. On every attach and detach, the planner broadcasts the payload's fitted spheres on `<node>/attached_spheres`; `robot_segmentation` writes them into its own copy of the robot kinematics, so the payload is carved out of the depth exactly like the arm itself is. **No `set_mask` call is needed for a grasped payload.**

Nothing is coupled by this: the planner publishes and never waits on, checks for, or depends on a subscriber — attach behaves identically whether or not `robot_segmentation` is running. The topic is `TRANSIENT_LOCAL`, so a segmentation node that starts (or respawns) mid-transport picks up the current payload immediately.

Two parameters on `robot_segmentation` govern it:

| Parameter | Default | Meaning |
|---|---|---|
| `attached_spheres_topic` | `/unified_planner/attached_spheres` | Set this if you renamed the planner node — a wrong value fails **silently**, as an absent subscription. |
| `payload_distance_threshold` | `0.02` | Keep-distance applied to the payload's spheres only. Deliberately tighter than the robot's own `distance_threshold`: those spheres were just fitted to a geometry you supplied, so they don't need the slack the links get for model error — and a 5 cm halo would erase the table from the map at the exact moment of placing onto it. |

Why this is stronger than a hand-written mask: the mask drops every point within `payload_distance_threshold` of a payload sphere — the spheres *dilated*. So no voxel can survive inside the planner's own collision model of the payload, **by construction**. A `set_mask` shape and the MORPHIT sphere fit are two independently specified geometries with no such relation, and a fit that bulges past a face leaves voxels inside the collision model — a collision the planner can never clear.

#### When you still want `set_mask`

`set_mask` is not obsolete. It remains the tool for two cases the automatic path cannot cover:

- **A payload larger than the sphere budget.** With `extra_collision_spheres: {attached_object: 4}`, a big part is masked as coarsely as it is modelled: its extremities stay in the voxel grid. Raise the budget, or add a `set_mask` shape on top.
- **Anything that is not the grasped payload** — a fixture, a table region, a static nuisance in view. There is no `attach` equivalent.

```bash
ros2 service call /set_mask curobo_msgs/srv/SetMask \
  "{name: 'payload', type: 0, frame_id: 'dsr01/link6', \
    pose: {position: {x: 0.0, y: 0.0, z: 0.05}, orientation: {w: 1.0}}, \
    dimensions: {x: 0.04, y: 0.04, z: 0.04}}"
```

It is nearly the same call as `attach_object` above (`type`, `pose`, `dimensions`, and the same frame convention). One difference matters: `set_mask` re-resolves its `frame_id` **every cycle**, so the mask tracks the arm continuously, whereas the attached spheres follow the arm through its own kinematics — the same visual result by a different mechanism. Call `remove_mask` with the same name when you're done.

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
| Attach fails | Robot config lacks `attached_object`, dimensions are non-positive, the orientation quaternion is `(0,0,0,0)`, `header.frame_id` isn't a valid TF frame, or something is already attached (detach first) |
| Adding objects fails after many adds | Collision cache full — grow it (see above) |

## Next steps

- [Tutorial 7: Camera-Based Obstacle Detection](07-pointcloud-detection.md) — automatic obstacles from depth cameras
- [Tutorial 4: Robot Execution](04-robot-execution.md)

[← Tutorial 2](02-adding-your-robot.md) | [Tutorial 4 →](04-robot-execution.md)
