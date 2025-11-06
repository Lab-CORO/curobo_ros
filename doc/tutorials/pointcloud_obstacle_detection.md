# Point Cloud Obstacle Detection Pipeline

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

### 2. Launch trajectory generation

Start the trajectory generation node with point cloud camera enabled:

```bash
ros2 launch curobo_ros gen_traj.launch.py use_pointcloud_camera:=true
```

### 3. Generate trajectories

The trajectory generation automatically updates the world model from cameras before planning:

```bash
ros2 service call /curobo_gen_traj/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
  "{target_pose: {position: {x: 0.5, y: 0.2, z: 0.3}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}"
```

### 4. Manual world update (optional)

You can manually trigger a world update from cameras:

```bash
ros2 service call /curobo_gen_traj/update_world_from_cameras std_srvs/srv/Trigger
```

## Services

| Service Name | Service Type | Description |
|-------------|-------------|-------------|
| `/curobo_gen_traj/update_world_from_cameras` | `Trigger` | Manually update world model from all camera observations |

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
   ros2 topic echo /masked_pointcloud
   ```

2. Verify camera is ready:
   ```bash
   ros2 service call /curobo_gen_traj/update_world_from_cameras std_srvs/srv/Trigger
   ```
   Should return: `success: True` with camera names

3. Check camera pose is correct - the virtual camera should "see" the workspace

### Trajectories collide with obstacles

1. Reduce `voxel_size` for finer resolution (but slower)
2. Increase `collision_activation_distance` for more conservative planning
3. Verify point cloud quality and filtering

### Performance issues

1. Increase `voxel_size` for faster processing
2. Reduce point cloud density in robot_segmentation
3. Check GPU memory usage

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
- [Motion Generation](./doosan_example.md)
- [Adding Collision Objects](./adding_collision_objects.md)
