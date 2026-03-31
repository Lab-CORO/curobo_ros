# Tutorial 7: Camera-Based Obstacle Detection

🟡 **Difficulty**: Intermediate
⏱️ **Estimated Time**: 30-45 minutes

## Overview

curobo_ros can integrate depth camera data directly into the collision checker. As new depth frames arrive, the BLOX world model is updated in real time, and any subsequent `generate_trajectory` call plans around the detected obstacles.

The recommended strategy is `depth_camera`, which subscribes to a depth image topic and feeds frames into cuRobo's BLOX voxel world. A `point_cloud` strategy also exists but is experimental.

---

## Architecture

```
Depth Camera (RealSense, Azure Kinect, etc.)
       │  /depth_to_rgb/image_raw  (raw — includes robot body)
       ▼
robot_segmentation node             (recommended — see Section 6)
  ├── joint states → collision spheres (CudaRobotModel)
  └── removes robot pixels → /masked_depth_image
       │
       ▼
DepthMapCameraStrategy              (configured in cameras.yaml)
  ├── intrinsics: camera_info topic or hardcoded
  ├── extrinsics: config [x,y,z,qw,qx,qy,qz] or TF
  └── each frame → add to cuRobo BLOX world model
       │
       ▼
World Model (BLOX voxels)
       │
       ▼
generate_trajectory → collision-free planning
```

The world model update happens inside the depth image callback — there is no manual trigger required. Each new depth frame automatically updates the voxel occupancy.

---

## Prerequisites

- Completed [Tutorial 1: Your First Trajectory](01-first-trajectory.md)
- A depth camera publishing to a ROS2 topic (`sensor_msgs/Image`, encoding `16UC1` or `32FC1`)
- Camera info topic available (`sensor_msgs/CameraInfo`) if you are not hardcoding intrinsics
- TF transform available for `base_link → camera_frame` if you are not hardcoding extrinsics

---

## Section 1: Camera Configuration File

Cameras are configured in a YAML file and passed to the node at launch via `cameras_config_file`.

### 1.1 File Structure

```yaml
cameras:
  - name: <unique_camera_name>
    type: depth_camera          # recommended; 'point_cloud' is experimental
    topic: <depth_image_topic>
    frame_id: <camera_tf_frame>
    camera_info: <camera_info_topic>   # used if intrinsics are empty
    intrinsics: []              # empty = read from camera_info topic
    extrinsics: []              # empty = read from TF (base_link → frame_id)
```

### 1.2 Example: Single RealSense / Azure Kinect Camera

```yaml
cameras:
  - name: depth_camera_front
    type: depth_camera
    topic: /depth_to_rgb/image_raw
    frame_id: rgb_camera_link
    camera_info: /depth_to_rgb/camera_info
    intrinsics: []
    extrinsics: [0.161, -0.428, 1.585, -0.3409296, 0.6840516, -0.6215661, 0.1717437]
```

### 1.3 Intrinsics Formats

**Option A: read from `camera_info` topic (recommended)**

```yaml
intrinsics: []
camera_info: /depth_to_rgb/camera_info
```

The node waits up to 5 seconds at startup for a `CameraInfo` message on that topic.

**Option B: 9-element K matrix**

The K matrix is flattened row-major: `[fx, 0, cx, 0, fy, cy, 0, 0, 1]`

```yaml
intrinsics: [615.3, 0.0, 320.5, 0.0, 615.8, 240.5, 0.0, 0.0, 1.0]
```

### 1.4 Extrinsics Formats

Extrinsics define the camera pose in the robot's base frame.

**Option A: read from TF at runtime (recommended for dynamic setups)**

```yaml
extrinsics: []
frame_id: rgb_camera_link   # TF lookup: base_link → rgb_camera_link
```

**Option B: 7-element list `[x, y, z, qw, qx, qy, qz]`**

```yaml
extrinsics: [0.161, -0.428, 1.585, -0.3409296, 0.6840516, -0.6215661, 0.1717437]
```


> **Note**: When static extrinsics are provided in the config, TF is not queried. Static extrinsics are preferred for fixed-mount cameras.

---

## Section 2: Launch with Camera

Pass the YAML file to the launch command:

```bash
ros2 launch curobo_ros gen_traj.launch.py \
  cameras_config_file:=/path/to/cameras.yaml
```

A default example file is provided at `config/cameras.yaml` inside the `curobo_ros` package.

```bash
ros2 launch curobo_ros gen_traj.launch.py \
  cameras_config_file:=$(ros2 pkg prefix curobo_ros)/share/curobo_ros/config/cameras.yaml
```

At startup, the node logs each camera that is successfully initialized:

```
[INFO] Loading camera configuration from: /path/to/cameras.yaml
[INFO] Added camera strategy 'depth_camera_front' of type 'depth_camera'
[INFO] Camera intrinsics from topic: fx=615.30, fy=615.80, cx=320.50, cy=240.50
[INFO] Using static extrinsics from config file
[INFO] DepthMap camera initialized with depth topic: /depth_to_rgb/image_raw
```

---

## Section 3: How the World Model Updates

Once the camera is initialized, it subscribes to the depth topic. For each incoming frame:

1. The depth image is converted to a float tensor (mm → m if `16UC1`)
2. The camera pose is applied (static from config, or queried from TF)
3. The frame is added to the cuRobo BLOX world model:
   - `world_model.add_camera_frame(observation, "world")`
   - `world_model.process_camera_frames("world")`
   - `world_model.update_blox_hashes()`

This happens automatically on every depth frame. There is no manual trigger service needed.

When `generate_trajectory` is called, the world model already reflects the latest camera observations.

---

## Section 4: Verifying the Pipeline

### Check the camera is initialized

```bash
ros2 node list | grep unified_planner
ros2 service call /unified_planner/is_available std_srvs/srv/Trigger
```

### Check depth topic is publishing

```bash
ros2 topic hz /depth_to_rgb/image_raw
ros2 topic info /depth_to_rgb/image_raw
```

Expected: `sensor_msgs/msg/Image` at your camera's configured rate.

### Inspect the voxel grid

```bash
ros2 service call /unified_planner/get_voxel_grid curobo_msgs/srv/GetVoxelGrid
```

The number of occupied voxels (`data` array) should increase when obstacles are placed in front of the camera.

### Check collision distance

```bash
ros2 service call /unified_planner/get_collision_distance curobo_msgs/srv/GetCollisionDistance
```

Positive values = distance to nearest obstacle. Negative values = collision (penetration).

### Visualize voxels in RViz

You can vizualise collision on rviz with curobo_rviz panel.   

Then in RViz: **Add → MarkerArray → `/visualization_marker_voxel`**

---

## Section 5: Multiple Cameras

Multiple cameras can be declared in the same YAML file:

```yaml
cameras:
  - name: camera_front
    type: depth_camera
    topic: /front_camera/depth/image_raw
    frame_id: front_camera_link
    camera_info: /front_camera/depth/camera_info
    intrinsics: []
    extrinsics: []

  - name: camera_top
    type: depth_camera
    topic: /top_camera/depth/image_raw
    frame_id: top_camera_link
    camera_info: /top_camera/depth/camera_info
    intrinsics: []
    extrinsics: [0.0, 0.0, 1.5, 1.0, 0.0, 0.0, 0.0]
```

Each camera runs independently. If one camera fails to initialize, the others continue. The world model accumulates observations from all ready cameras.

---

## Section 6: Robot Segmentation

Without filtering, the camera sees the robot's own body as obstacles in the depth image. The planner detects a collision at the current configuration and refuses to plan. The `robot_segmentation` node removes the robot from the depth image before it reaches the collision checker.

### 6.1 How It Works

At 100 Hz, the node:

1. Reads the current joint positions from the robot
2. Computes the robot's collision spheres for that configuration (GPU via CudaRobotModel)
3. Converts the raw depth image into a 3D point cloud (using camera intrinsics)
4. Removes every point that falls within `distance_threshold` meters of any collision sphere
5. Converts the filtered point cloud back to a depth image
6. Publishes the cleaned image on `/masked_depth_image`

```
/depth_to_rgb/image_raw  ──►  robot_segmentation  ──►  /masked_depth_image
                                      ▲
                               joint states → collision spheres
```

The `cameras.yaml` must then point to `/masked_depth_image` instead of the raw camera topic so that the planner receives pre-filtered frames.

### 6.2 Launch

The node is a separate executable that runs alongside the planner:

```bash
# Terminal 1: planner + camera
ros2 launch curobo_ros gen_traj.launch.py \
  cameras_config_file:=/path/to/cameras.yaml

# Terminal 2: robot segmentation
ros2 run curobo_ros robot_segmentation
```

### 6.3 Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_config_file` | `m1013.yml` (package default) | Path to robot YAML — must match the planner's config |
| `depth_image_topic` | `/depth_to_rgb/image_raw` | Raw depth image input |
| `camera_info_topic` | `/depth_to_rgb/camera_info` | Intrinsics topic for 3D projection |
| `joint_states_topic` | `/dsr01/joint_states` | Robot joint positions |
| `robot_base_frame` | `base_link` | Frame for collision sphere projection |

Override any parameter with `--ros-args`:

```bash
ros2 run curobo_ros robot_segmentation --ros-args \
  -p robot_config_file:=/path/to/my_robot.yml \
  -p depth_image_topic:=/camera/depth/image_rect_raw \
  -p camera_info_topic:=/camera/depth/camera_info \
  -p joint_states_topic:=/my_robot/joint_states \
  -p robot_base_frame:=base_link
```

> **`distance_threshold`** (default: `0.05 m`) controls how much margin is added around each collision sphere. It is a constructor argument and cannot be changed at runtime. Increase it if the robot is occasionally self-detected due to calibration error.

### 6.4 Topics

**Subscriptions:**

| Topic | Type | Description |
|-------|------|-------------|
| `<depth_image_topic>` | `sensor_msgs/Image` | Raw depth image (`16UC1` mm or `32FC1` m) |
| `<camera_info_topic>` | `sensor_msgs/CameraInfo` | Intrinsics for depth→3D projection |

**Publications:**

| Topic | Type | Description |
|-------|------|-------------|
| `/masked_depth_image` | `sensor_msgs/Image` | Filtered depth image — robot pixels zeroed (`16UC1`) |
| `/collision_spheres` | `visualization_msgs/MarkerArray` | Robot collision spheres at current config (RViz debug) |
| `/robot_pointcloud_debug` | `sensor_msgs/PointCloud2` | Points that were removed (RViz debug, only published if subscribed) |

### 6.5 Integration with cameras.yaml

Point the `depth_camera` entry at the segmentation output:

```yaml
cameras:
  - name: depth_camera_front
    type: depth_camera
    topic: /masked_depth_image          # ← output of robot_segmentation
    frame_id: rgb_camera_link
    camera_info: /depth_to_rgb/camera_info
    intrinsics: []
    extrinsics: [0.161, -0.428, 1.585, -0.3409296, 0.6840516, -0.6215661, 0.1717437]
```

> The `camera_info` field still points to the **original** camera info topic — the segmentation node does not alter intrinsics, only pixel values.

### 6.6 Verify the Segmentation

```bash
# Check the filtered image is publishing
ros2 topic hz /masked_depth_image

# Visualize in RViz:
#   Add → Image → /masked_depth_image   (should show black pixels where the robot is)
#   Add → MarkerArray → /collision_spheres  (red spheres around robot links)
#   Add → PointCloud2 → /robot_pointcloud_debug  (points removed by segmentation)
```

If `robot_pointcloud_debug` shows points clustered around the robot's links and the masked image has black patches in those areas, segmentation is working correctly.

---

## Section 7: Tuning Parameters

Voxel resolution and collision safety margin can be adjusted while the node is running:

```bash
# Finer resolution (slower, more accurate)
ros2 param set /unified_planner voxel_size 0.02
ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger

# Coarser resolution (faster, less memory)
ros2 param set /unified_planner voxel_size 0.08
ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger

# Increase collision safety margin
ros2 param set /unified_planner collision_activation_distance 0.04
ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger
```

| `voxel_size` | Resolution | Speed | Memory |
|---|---|---|---|
| 0.02 m | Fine (2 cm) | Slow | High |
| 0.05 m | Default (5 cm) | Moderate | Moderate |
| 0.08 m | Coarse (8 cm) | Fast | Low |

---

## Troubleshooting

### Camera not initialized at startup

**Symptoms**: No log line `Added camera strategy '...'`

1. Verify `cameras_config_file` path is correct and the file is readable
2. Check the YAML is valid (no indentation errors)
3. Ensure the `cameras:` key is present with at least one entry

### Intrinsics error at startup

**Symptoms**: `Failed to receive camera info from /your/topic`

The node waits 5 seconds for a `CameraInfo` message. If the camera is not publishing yet:
- Start the camera driver before the planner, or
- Hardcode intrinsics in the YAML (Option B or C in Section 1.3)

### Obstacles not detected in trajectory planning

1. Confirm depth frames are arriving: `ros2 topic hz <depth_topic>`
2. Inspect the voxel grid — it should show occupied voxels near the obstacle
3. Check `frame_id` is correct — TF lookup failure falls back to identity pose (obstacles at wrong location)
4. Reduce `voxel_size` for finer detection
5. Increase `collision_activation_distance` so the planner avoids obstacles with more margin

### TF lookup failure

**Symptoms**: `Could not transform base_link to <frame_id>`

Either:
- Provide static extrinsics in the YAML config to bypass TF, or
- Ensure the camera TF frame is published (check `ros2 run tf2_ros tf2_echo base_link <frame_id>`)

### Planner rejects all trajectories (collision at start)

**Symptoms**: `generate_trajectory` always returns `success: false`, even for simple goals

The most likely cause is that the robot's own body is visible in the depth image and is detected as an obstacle. The planner sees a collision at the current state and cannot plan.

**Solution**: Run `robot_segmentation` and point `cameras.yaml` to `/masked_depth_image` (see Section 6).

**Verify**: Check the voxel grid with and without `robot_segmentation` running:

```bash
ros2 service call /unified_planner/get_voxel_grid curobo_msgs/srv/GetVoxelGrid
```

With segmentation, the number of occupied voxels near the robot's body should drop significantly.

### robot_segmentation not publishing

**Symptoms**: `/masked_depth_image` has no publishers or very low rate

1. Check that both the depth image and camera info topics are publishing:
   ```bash
   ros2 topic hz /depth_to_rgb/image_raw
   ros2 topic hz /depth_to_rgb/camera_info
   ```
2. Check that joint states are arriving (without them the timer callback does nothing):
   ```bash
   ros2 topic hz /dsr01/joint_states
   ```
3. If the joint states topic is different, override it:
   ```bash
   ros2 run curobo_ros robot_segmentation --ros-args \
     -p joint_states_topic:=/my_robot/joint_states
   ```

---

## Related Documentation

- [ROS Interfaces Reference — Parameters](../concepts/ros-interfaces.md#parameters)
- [Parameters Guide — voxel_size, collision_activation_distance](../concepts/parameters.md)
- [Tutorial 3: Collision Objects](03-collision-objects.md)

---

[← Tutorial 6: IK/FK Services](06-ik-fk-services.md) | [← Back to Tutorials](index.md)