# Exemple with doosan M1013


## 1. Motion Generation

The motion generation feature is provided from the node `curobo_gen_traj` an launch file is available: `launch/curobo_gen_traj.launch`. 
- To generate a trajectory, a message have to be send on the topic `marker_pose` and every 0.1s the node generate a trajectory if the `marker_pose` message is new (this part should be improve and replace by a service)
- A camera can be used to real world obstacles. 
- The trajectory is published on the topic `trajectory` and can be visualized in RViz with the package trajectory_preview. 
- Some parameters can be changed:
  - Some can be changed online 
    -  `max_attempts`
    -  `timeout`
    -  `time_dilation_factor`
  - Others have to be changed and the configuration should be update:
    - `voxel_size`
    - `collision_activation_distance`

| Service Name |Service Type | Description | Callback Function |
|-------------|-------------|-------------|------------------|
| `<node_name>/update_motion_gen_config` | `Trigger` | Updates the motion generation configuration to adjust planning parameters. | `set_motion_gen_config` |


|Topics Name |Publisher or Subscriber | Topic Type | Description | Callback Function |
|-------------|-------------|-------------|------------------|------------------|
|`marker_pose` |Subscriber | `PoseStamped` | Subscribe to the current pose of the marker. To publish a trajectory in the next 0.1s | `callback_marker_pose` |
|`/camera/camera/depth/image_rect_raw`| Subscriber | `sensor_msgs/Image` | Subscribes to the depth image from the camera. | `callback_depth_image` |
|`/camera/camera/depth/camera_info`| Subscriber | `sensor_msgs/CameraInfo` | Subscribes to the camera information. | `callback_camera_info` |
|`/trajectory`| Publisher | `trajectory_msgs/JointTrajectory` | Publishes the generated trajectory. | |

---



