# Tutorial 1: Your First Trajectory

**Difficulty**: Beginner · **Time**: ~20 min · **Prerequisites**: [Installation](../getting-started/installation.md) complete

You will plan a motion from the command line, read the response, preview it in RViz, and execute it on the emulated robot.

## 1. Start the system

In the container:

```bash
ros2 launch curobo_ros gen_traj.launch.py robot:=emulator world_file:=$(ros2 pkg prefix curobo_ros)/share/curobo_ros/config/floor_world.yml
```

`robot:=emulator` runs the Doosan M1013 kinematics without hardware. The `world_file` adds a floor — without it the world is completely empty and the planner will happily swing the arm below the base.

Wait for warmup (25–35 s). The node is ready when:

```bash
ros2 param get /unified_planner node_is_available
# Boolean value is: True
```

Open a second shell in the container for the commands below:

```bash
docker exec -it curobo_ampere_dev bash
source /home/ros2_ws/install/setup.bash
```

## 2. Know your workspace

The default robot is a **Doosan M1013**: 6 DOF, ~1.3 m reach, base frame `base_0`. A comfortable target zone for this tutorial:

- x: 0.3 to 0.9 m (in front of the robot)
- y: −0.5 to 0.5 m
- z: 0.1 to 0.8 m (above the base)

Orientation is a quaternion in the base frame. `{x: 0, y: 1, z: 0, w: 0}` points the tool straight down — a good default for tabletop poses.

## 3. Generate a trajectory

```bash
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
  "{target_pose: {position: {x: 0.5, y: 0.2, z: 0.4}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}"
```

A successful response looks like:

```
success: True
message: '...'
trajectory: [<sensor_msgs/JointState with position and velocity>, ...]
dt: 0.025
```

The `trajectory` field is an array of `JointState` waypoints; `dt` is the time step between them (the `interpolation_dt` parameter). In RViz, the translucent **preview robot** replays the plan — nothing has moved yet.

Things to try:

- an unreachable pose (`x: 2.0`) → `success: False` with an IK error in `message`;
- different orientations — e.g. `{x: 0, y: 0, z: 0, w: 1}` (tool pointing up along +z).

## 4. Execute it

Planning and execution are separate. Execution goes through the `execute_trajectory` action; on the emulator the "robot" is simulated, so this is always safe:

```bash
ros2 action send_goal /unified_planner/execute_trajectory curobo_msgs/action/SendTrajectory \
  "{target_pose: {position: {x: 0.5, y: 0.2, z: 0.4}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}" --feedback
```

Because the goal matches the plan you just generated (and `allow_cached` defaults to `true`), the action reuses the cached trajectory instead of re-planning, then streams it. Feedback shows `state: EXECUTING` and `step_progression` climbing from 0 to 1; the main RViz robot follows.

You can also skip step 3 entirely — the action plans by itself when there is no cached trajectory.

## 5. Add an obstacle and re-plan

```bash
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject \
  "{type: 0, name: 'table', pose: {position: {x: 0.6, y: 0.0, z: 0.1}, orientation: {w: 1.0}}, \
    dimensions: {x: 0.8, y: 1.0, z: 0.05}, color: {r: 0.5, g: 0.3, b: 0.1, a: 1.0}}"
```

This adds a 0.8 × 1.0 × 0.05 m tabletop centred 0.6 m in front of the base at z = 0.1 m (dimensions are full extents for a cuboid). It appears in RViz via the `/unified_planner/scene_obstacles` markers. Re-run the `generate_trajectory` call from step 3: the new plan curves around the table.

List and clean up:

```bash
ros2 service call /unified_planner/get_obstacles std_srvs/srv/Trigger
ros2 service call /unified_planner/remove_object curobo_msgs/srv/RemoveObject "{name: 'table'}"
```

Obstacle types, dimension semantics, meshes, and attachment are covered in [Tutorial 3](03-collision-objects.md).

## 6. Tune a parameter

Two examples that illustrate the two kinds of parameters (full list in the [Parameters Guide](../concepts/parameters.md)):

```bash
# Plan-time: takes effect on the next request, no rebuild
ros2 param set /unified_planner max_attempts 3

# Build-time: requires a solver rebuild (~20 s, blocking)
ros2 param set /unified_planner voxel_size 0.03
ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger
```

## Common issues

| Symptom | Likely cause | Fix |
|---|---|---|
| `success: False`, IK error | Pose out of reach or in collision | Bring the target into the zone from step 2; check obstacles |
| Service call hangs forever | Node still warming up | Wait for `node_is_available` to be `True` |
| Plan goes through an obstacle you see in RViz | Marker ≠ collision world | Confirm with `get_obstacles`; markers refresh every 2 s |
| First plan after switching planners is slow | CUDA graph re-capture | Expected — subsequent plans are fast |

## Next steps

- [Tutorial 3: Managing Collision Objects](03-collision-objects.md)
- [Tutorial 4: Robot Execution](04-robot-execution.md) — run it on a real robot
- [RViz Plugin](../concepts/rviz-plugin.md) — the same workflow with mouse and panels

[← Tutorials index](index.md) | [Tutorial 2 →](02-adding-your-robot.md)
