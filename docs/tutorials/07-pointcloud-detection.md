# Point Cloud Obstacle Detection Pipeline

This section is not working.

This tutorial explains how to use the point cloud from `robot_segmentation` to detect obstacles for cuRobo trajectory generation.

## Overview

The pipeline integrates point cloud data into cuRobo's collision checking system using a camera strategy pattern. The flow is:

1. **robot_segmentation node** - Segments the robot from the point cloud and publishes a filtered point cloud
2. **PointCloudCameraStrategy** - Converts the point cloud into a synthetic depth image
3. **CameraContext** - Manages multiple camera sources
4. **ConfigWrapperMotion** - Integrates camera observations into the world model
5. **generate_trajectory node** - Uses the updated world model for collision-free planning

## Architecture

```
┌─────────────────────┐
│ robot_segmentation  │ Publishes /masked_pointcloud
└──────────┬──────────┘
           │
           v
┌─────────────────────────────┐
│ PointCloudCameraStrategy    │ Converts to synthetic depth image
└──────────┬──────────────────┘
           │
           v
┌──────────────────────┐
│   CameraContext      │ Manages camera observations
└──────────┬───────────┘
           │
           v
┌──────────────────────────┐
│  ConfigWrapperMotion     │ Updates cuRobo world model
└──────────┬───────────────┘
           │
           v
┌──────────────────────┐
│ generate_trajectory  │ Plans collision-free trajectories
└──────────────────────┘
```

## Components

### 1. CameraStrategy (Base Class)

Abstract base class that defines the interface for all camera strategies:

- `get_camera_observation()` - Returns a CameraObservation for cuRobo
- `is_ready()` - Checks if camera has data available
- `get_camera_pose()` - Returns camera pose in world frame

### 2. PointCloudCameraStrategy

Converts point cloud data to camera observations:

```python
from curobo_ros.cameras import PointCloudCameraStrategy

# Initialize with custom parameters
pointcloud_camera = PointCloudCameraStrategy(
    node,
    pointcloud_topic='/masked_pointcloud',
    camera_pose=[0.0, 0.0, 1.5, 1.0, 0.0, 0.0, 0.0],  # [x, y, z, qw, qx, qy, qz]
    image_width=640,
    image_height=480,
    fx=525.0,  # Focal length x
    fy=525.0,  # Focal length y
    cx=319.5,  # Principal point x
    cy=239.5   # Principal point y
)
```

The strategy:
- Subscribes to a PointCloud2 topic
- Projects 3D points onto a 2D image plane
- Creates a synthetic depth image
- Wraps it in a CameraObservation for cuRobo

### 3. CameraContext

Manages multiple camera strategies:

```python
from curobo_ros.cameras import CameraContext

camera_context = CameraContext(node)

# Add cameras
camera_context.add_camera('pointcloud', pointcloud_camera)
camera_context.add_camera('realsense', realsense_camera)

# Update world model with all cameras
camera_context.update_world_collision_model(world_model, "world")
```

### 4. ConfigWrapperMotion Integration

The `ConfigWrapperMotion` class now automatically:
- Initializes a CameraContext
- Adds cameras based on ROS parameters
- Updates the world model before trajectory planning

## Configuration

### ROS2 Parameters

Configure the point cloud camera in your launch file:

```python
parameters=[{
    'use_pointcloud_camera': True,
    'pointcloud_topic': '/masked_pointcloud',
    'camera_pose': [0.0, 0.0, 1.5, 1.0, 0.0, 0.0, 0.0],  # Virtual camera position
    'voxel_size': 0.05,
    'collision_activation_distance': 0.025,
}]
```

Parameters:
- `use_pointcloud_camera` (bool): Enable/disable point cloud camera
- `pointcloud_topic` (string): Topic name for the point cloud
- `camera_pose` (array): Camera pose [x, y, z, qw, qx, qy, qz] in world frame
- `voxel_size` (float): Size of voxels for BLOX collision checking
- `collision_activation_distance` (float): Distance threshold for collision detection

## Usage

### 1. Launch robot_segmentation

Start the robot segmentation node to filter the robot from the point cloud:

```bash
ros2 run curobo_ros robot_segmentation
```

This publishes the filtered point cloud on `/masked_pointcloud`.

**Verify it's working:**

```bash
# Check if segmentation node is running
ros2 node list | grep robot_segmentation

# Inspect published point cloud
ros2 topic info /masked_pointcloud

# Expected output: Type sensor_msgs/msg/PointCloud2

# Check point cloud data rate
ros2 topic hz /masked_pointcloud

# Expected: ~5-30 Hz depending on camera

# View point cloud statistics
ros2 topic echo /masked_pointcloud --once | head -20

# Shows header, dimensions, and point count
```

### 2. Launch trajectory generation

Start the trajectory generation node with point cloud camera enabled:

```bash
ros2 launch curobo_ros gen_traj.launch.py use_pointcloud_camera:=true
```

### 3. Generate trajectories

The trajectory generation automatically updates the world model from cameras before planning:

```bash
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
  "{target_pose: {position: {x: 0.5, y: 0.2, z: 0.3}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}"
```

### 4. Manual world update (optional)

You can manually trigger a world update from cameras:

```bash
ros2 service call /unified_planner/update_world_from_cameras std_srvs/srv/Trigger
```

### 5. Debug Camera Pipeline

Use these CLI commands to inspect and debug the camera observation pipeline:

```bash
# 1. Check camera is ready
ros2 service call /unified_planner/update_world_from_cameras std_srvs/srv/Trigger

# Response includes camera names that provided observations
# Example: message: "Updated from cameras: pointcloud"

# 2. Inspect voxel grid
ros2 service call /unified_planner/get_voxel_grid curobo_msgs/srv/GetVoxelGrid

# Shows voxelized environment from camera observations

# 3. Visualize voxels in RViz
ros2 run curobo_ros viz_voxel_grid &

# Then in RViz, add MarkerArray display for /visualization_marker_voxel

# 4. Check collision distance with obstacles
ros2 service call /unified_planner/get_collision_distance curobo_msgs/srv/GetCollisionDistance

# Positive values = distance to obstacle (meters)
# Negative values = penetration (collision!)

# 5. Monitor camera observation rate
ros2 topic hz /camera/depth/image_raw

# Should match your camera's configured rate

# 6. Check camera TF transforms
ros2 run tf2_ros tf2_echo world camera_link

# Verifies camera pose is correct

# 7. List all active cameras
ros2 param get /unified_planner use_pointcloud_camera

# Shows if point cloud camera is enabled

# 8. Verify point cloud in world frame
ros2 run tf2_ros tf2_echo world <point_cloud_frame_id>

# Checks if point cloud frame is properly linked to world
```

### 6. Performance Monitoring

Monitor the performance of the point cloud detection pipeline:

```bash
# 1. Check planning time with obstacles
ros2 topic echo /unified_planner/planning_time

# Should be < 100ms for typical scenes

# 2. Monitor GPU memory usage (if CUDA)
nvidia-smi -l 1

# Watch GPU memory while adding obstacles

# 3. Check node CPU usage
top -p $(pgrep -f unified_planner)

# Monitor CPU % during trajectory generation

# 4. Measure end-to-end latency
time ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
  "{target_pose: {position: {x: 0.5, y: 0.2, z: 0.3}, orientation: {w: 1.0}}}"

# Includes camera update + planning time

# 5. Check point cloud processing rate
ros2 topic bw /masked_pointcloud

# Shows bandwidth usage (KB/s)
```

## Services

| Service Name | Service Type | Description |
|-------------|-------------|-------------|
| `/unified_planner/update_world_from_cameras` | `Trigger` | Manually update world model from all camera observations |

## Advanced: Adding Custom Camera Strategies

You can create custom camera strategies by inheriting from `CameraStrategy`:

```python
from curobo_ros.cameras import CameraStrategy
from curobo.types.camera import CameraObservation

class MyCustomCameraStrategy(CameraStrategy):
    def __init__(self, node):
        super().__init__(node)
        # Your initialization

    def get_camera_observation(self) -> CameraObservation:
        # Return your camera observation
        pass

    def is_ready(self) -> bool:
        # Check if data is available
        pass

    def get_camera_pose(self) -> Pose:
        # Return camera pose
        pass
```

Then add it to the CameraContext:

```python
custom_camera = MyCustomCameraStrategy(node)
camera_context.add_camera('my_custom', custom_camera)
```

## Troubleshooting

### No obstacles detected

1. Check that robot_segmentation is publishing:
   ```bash
   ros2 topic echo /masked_pointcloud --once

   # If no output, check if node is running
   ros2 node list | grep robot_segmentation

   # Check source point cloud topic
   ros2 topic list | grep points

   # Verify source camera is publishing
   ros2 topic hz /camera/depth/points
   ```

2. Verify camera is ready:
   ```bash
   ros2 service call /unified_planner/update_world_from_cameras std_srvs/srv/Trigger

   # Should return: success: True with camera names
   # Example: message: "Updated from cameras: pointcloud"

   # If no cameras listed, check parameter
   ros2 param get /unified_planner use_pointcloud_camera

   # Should return: bool value: True
   ```

3. Check camera pose is correct - the virtual camera should "see" the workspace:
   ```bash
   # View current camera pose parameter
   ros2 param get /unified_planner camera_pose

   # Check TF transform for camera frame
   ros2 run tf2_ros tf2_echo world camera_link

   # Visualize camera frustum in RViz
   # Add Camera display and set topic to camera info
   ```

4. Verify voxelization is working:
   ```bash
   # Get voxel grid
   ros2 service call /unified_planner/get_voxel_grid curobo_msgs/srv/GetVoxelGrid

   # Check number of occupied voxels (should be > 0)

   # Visualize voxels
   ros2 run curobo_ros viz_voxel_grid

   # In RViz: Add → MarkerArray → /visualization_marker_voxel
   ```

### Trajectories collide with obstacles

1. Reduce `voxel_size` for finer resolution (but slower):
   ```bash
   # Check current voxel size
   ros2 param get /unified_planner voxel_size

   # Set smaller voxel size for finer resolution
   ros2 param set /unified_planner voxel_size 0.02

   # Apply changes
   ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger

   # Test trajectory again
   ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration "{...}"
   ```

2. Increase `collision_activation_distance` for more conservative planning:
   ```bash
   # Check current collision distance
   ros2 param get /unified_planner collision_activation_distance

   # Increase safety margin
   ros2 param set /unified_planner collision_activation_distance 0.04

   # Apply and test
   ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger
   ```

3. Verify point cloud quality and filtering:
   ```bash
   # Check point cloud statistics
   ros2 topic echo /masked_pointcloud --once | grep -E "(width|height|point_step)"

   # Count points in cloud
   ros2 topic echo /masked_pointcloud --once | grep "is_dense"

   # Visualize filtered cloud in RViz
   # Add → PointCloud2 → /masked_pointcloud
   ```

### Performance issues

1. Increase `voxel_size` for faster processing:
   ```bash
   # Current voxel size
   ros2 param get /unified_planner voxel_size

   # Increase for faster performance
   ros2 param set /unified_planner voxel_size 0.08

   # Apply changes
   ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger

   # Measure planning time improvement
   time ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration "{...}"
   ```

2. Reduce point cloud density in robot_segmentation:
   ```bash
   # Check current point cloud size
   ros2 topic echo /masked_pointcloud --once | grep "width"

   # If using voxel grid filter, adjust leaf size parameter
   ros2 param list | grep voxel_leaf_size

   # Restart with larger leaf size for fewer points
   ```

3. Check GPU memory usage:
   ```bash
   # Monitor GPU memory
   watch -n 1 nvidia-smi

   # Check specific process memory
   nvidia-smi --query-compute-apps=pid,used_memory --format=csv

   # If out of memory, reduce cache sizes
   ros2 service call /unified_planner/set_collision_cache \
     curobo_msgs/srv/SetCollisionCache "{obb: 100, mesh: -1, blox: 20}"

   ros2 service call /unified_planner/update_motion_gen_config std_srvs/srv/Trigger
   ```

4. Profile node performance:
   ```bash
   # Check CPU usage
   top -p $(pgrep -f unified_planner)

   # Monitor message processing delays
   ros2 topic delay /masked_pointcloud

   # Check service call latency
   time ros2 service call /unified_planner/update_world_from_cameras std_srvs/srv/Trigger

   # Analyze with ros2_tracing (if installed)
   ros2 run tracetools trace -o curobo_trace
   ```

## Examples

### Example 1: Point cloud from segmentation

```python
# In your launch file
Node(
    package='curobo_ros',
    executable='generate_trajectory',
    parameters=[{
        'use_pointcloud_camera': True,
        'pointcloud_topic': '/masked_pointcloud',
        'camera_pose': [0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
        'voxel_size': 0.05,
    }]
)
```

### Example 2: Multiple cameras

```python
# In ConfigWrapperMotion.__init__ after initialization
realsense_camera = RealsenseStrategy(
    node,
    depth_topic='/camera/depth/image_rect_raw',
    camera_info_topic='/camera/depth/camera_info',
    camera_pose=[0.5, 0.5, 1.0, 1.0, 0.0, 0.0, 0.0]
)
self.camera_context.add_camera('realsense', realsense_camera)
```

## Pipeline Complete Workflow

1. **Setup**: Launch robot_segmentation and generate_trajectory nodes
2. **Initialization**: PointCloudCameraStrategy subscribes to /masked_pointcloud
3. **Data Flow**:
   - robot_segmentation publishes filtered point cloud
   - PointCloudCameraStrategy receives and converts to depth image
   - CameraContext manages the observation
4. **Planning**: When trajectory is requested:
   - ConfigWrapperMotion calls update_world_from_cameras()
   - CameraContext gets observations from all ready cameras
   - World model is updated with BLOX voxels
   - Trajectory is planned avoiding detected obstacles
5. **Execution**: Trajectory is sent to robot

## See Also

- [Robot Segmentation](../core/robot_segmentation.py)
- [Camera Strategies](../../curobo_ros/cameras/)
- [Motion Generation](./examples/doosan-m1013.md)
- [Adding Collision Objects](./03-collision-objects.md)
