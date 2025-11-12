# Camera Configuration Guide

This guide explains how to configure cameras for cuRobo with flexible support for both TF-based and matrix-based pose/intrinsics.

## Overview

The camera system now supports **two configuration modes** for each camera parameter:

### Pose Configuration (Extrinsic Matrix)
1. **TF-based** (preferred): Uses ROS2 TF to lookup camera pose dynamically
   - Requires `frame_id` to be set
   - Most flexible - pose updates automatically if TF changes

2. **Matrix-based** (fallback): Uses explicit 4x4 extrinsic matrix
   - Requires `extrinsic_matrix` in config
   - Fixed pose - doesn't update if calibration changes
   - Useful when TF is unavailable or network isolated

### Intrinsics Configuration
1. **ROS Topic** (preferred): Subscribes to camera_info topic
   - Requires `camera_info` topic path
   - Automatically extracted K matrix

2. **Matrix-based** (fallback): Uses explicit 3x3 intrinsic matrix
   - Requires `intrinsic_matrix` in config
   - Fixed calibration - more reliable for static cameras

## Configuration Formats

### Depth Camera with TF (Recommended)
```yaml
cameras:
  - name: "my_camera"
    type: "depth_camera"
    topic: "/camera/depth/image_rect_raw"
    camera_info: "/camera/depth/camera_info"
    frame_id: "camera_depth_frame"
```

### Depth Camera with Extrinsic Matrix Only
```yaml
cameras:
  - name: "my_camera"
    type: "depth_camera"
    topic: "/camera/depth/image_rect_raw"
    camera_info: "/camera/depth/camera_info"
    frame_id: ""  # Empty = skip TF lookup
    extrinsic_matrix: [
      [1.0, 0.0, 0.0, 0.1],
      [0.0, 1.0, 0.0, 0.2],
      [0.0, 0.0, 1.0, 0.5],
      [0.0, 0.0, 0.0, 1.0]
    ]
```

### Depth Camera with Intrinsic Matrix Only
```yaml
cameras:
  - name: "my_camera"
    type: "depth_camera"
    topic: "/camera/depth/image_rect_raw"
    camera_info: ""  # Empty = skip ROS topic
    frame_id: "camera_frame"
    intrinsic_matrix: [
      [500.0, 0.0, 320.0],
      [0.0, 500.0, 240.0],
      [0.0, 0.0, 1.0]
    ]
```

### Depth Camera Fully Offline (No TF, No ROS Topics)
```yaml
cameras:
  - name: "my_camera"
    type: "depth_camera"
    topic: "/camera/depth"
    camera_info: ""
    frame_id: ""
    extrinsic_matrix: [
      [1.0, 0.0, 0.0, 0.1],
      [0.0, 1.0, 0.0, 0.2],
      [0.0, 0.0, 1.0, 0.5],
      [0.0, 0.0, 0.0, 1.0]
    ]
    intrinsic_matrix: [
      [500.0, 0.0, 320.0],
      [0.0, 500.0, 240.0],
      [0.0, 0.0, 1.0]
    ]
```

### Point Cloud Camera
```yaml
cameras:
  - name: "lidar"
    type: "point_cloud"
    topic: "/sensor/pointcloud"
    frame_id: "sensor_frame"  # TF-based
    pixel_size: 0.01
    # OR use extrinsic_matrix if TF unavailable:
    # extrinsic_matrix: [...]
```

## Matrix Formats

### Extrinsic Matrix (4x4 Homogeneous Transformation)
Represents the pose of the camera in the base frame:

```
[R11 R12 R13 tx]     [Rotation   Translation]
[R21 R22 R23 ty]  =  [Matrix     Vector    ]
[R31 R32 R33 tz]     [3x3        3x1       ]
[0   0   0   1 ]     [Homogeneous Coordinate]
```

**Translation** (first 3 elements of last column):
- tx, ty, tz: position in meters relative to base_link

**Rotation** (3x3 upper-left submatrix):
- Row vectors form the rotated coordinate axes
- Column vectors form the original axes in rotated coordinates

Example: Camera at (0.1m, 0.2m, 0.5m) with identity rotation:
```yaml
extrinsic_matrix: [
  [1.0, 0.0, 0.0, 0.1],
  [0.0, 1.0, 0.0, 0.2],
  [0.0, 0.0, 1.0, 0.5],
  [0.0, 0.0, 0.0, 1.0]
]
```

### Intrinsic Matrix (3x3 Camera Calibration)
Represents focal length and principal point:

```
[fx  0  cx]     [Focal Length X      Principal X]
[ 0 fy  cy]  =  [Focal Length Y      Principal Y]
[ 0  0   1]     [Homogeneous       Coordinate  ]
```

- `fx`, `fy`: Focal lengths in pixels
- `cx`, `cy`: Principal point (image center) in pixels

Example:
```yaml
intrinsic_matrix: [
  [500.0, 0.0, 320.0],    # fx=500, cx=320
  [0.0, 500.0, 240.0],    # fy=500, cy=240
  [0.0, 0.0, 1.0]
]
```

## Configuration Priority

For **pose** (extrinsic):
1. ✅ TF lookup using `frame_id` (dynamic, recommended)
2. ✅ Explicit `extrinsic_matrix` (static fallback)
3. ⚠️  Identity pose (if both unavailable)

For **intrinsics**:
1. ✅ ROS `camera_info` topic (recommended)
2. ✅ Explicit `intrinsic_matrix` (fallback)
3. ❌ Will fail if both unavailable (required)

## Logging

The system logs configuration status during initialization:

**Good - Using TF:**
```
Camera 'my_camera' has frame_id set - will use TF lookup
Camera intrinsics: fx=500.00, fy=500.00, cx=320.00, cy=240.00
```

**Good - Using Matrix:**
```
Camera 'my_camera' has no frame_id but has extrinsic_matrix defined
Using provided intrinsic_matrix: fx=500.00, fy=500.00, cx=320.00, cy=240.00
```

**Warning - Degraded Mode:**
```
Camera 'my_camera' has no frame_id and no extrinsic_matrix defined
Camera 'my_camera' has no camera_info and no intrinsic_matrix defined
```

## Best Practices

1. **Always provide intrinsics** (either topic or matrix)
   - Depth image processing requires camera calibration

2. **Prefer TF for pose** when possible
   - More flexible and maintainable
   - Allows runtime pose updates

3. **Use matrices for fixed installations**
   - Pre-calibrated hardware that won't change
   - Eliminates TF dependency for isolated systems

4. **Validate matrix format**
   - Extrinsic: exactly 4×4
   - Intrinsic: exactly 3×3
   - Use calibration tools (OpenCV, ROS calibration) to generate accurate values

5. **Test configuration during development**
   - Check logs for configuration messages
   - Verify camera data flows correctly
   - Validate poses visually in RViz if using TF

## Example Files

See `example_camera_config.yaml` for complete working examples of all configurations.
