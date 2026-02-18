# Ros2 Interfaces
Summarry of every interfaces for this project.

## Topics
| Interface Type | Name | Message/Service Type | Description |
|---|---|---|---|
|  Publisher | `<node_name>/collision_spheres` | `MarkerArray` | Publishes collision spheres used for collision avoidance. |
|  Publisher | `/trajectory` | `trajectory_msgs/JointTrajectory` | Publishes the generated trajectory. |

## Services

| Name | Message/Service Type | Description |
|---|---|---|
| `<node_name>/add_object` | [AddObject](https://github.com/Lab-CORO/curobo_msgs/blob/main/srv/AddObject.srv) | Adds a new object to the scene. |
| `<node_name>/remove_object` | [RemoveObject](https://github.com/Lab-CORO/curobo_msgs/blob/main/srv/RemoveObject.srv) | Removes a specific object from the scene. |
| `<node_name>/get_obstacles` | `Trigger` | Retrieves a list of all obstacles in the scene. |
| `<node_name>/remove_all_objects` | `Trigger` | Clears all objects from the scene. |
| `<node_name>/get_voxel_grid` | [GetVoxelGrid](https://github.com/Lab-CORO/curobo_msgs/blob/main/srv/GetVoxelGrid.srv) | Retrieves voxelized representation of the environment. |
| `<node_name>/get_collision_distance` | [GetCollisionDistance](https://github.com/Lab-CORO/curobo_msgs/blob/main/srv/GetCollisionDistance.srv) | Computes distance between objects and potential collisions. |
| `<node_name>/update_motion_gen_config` | `Trigger` | Updates motion generation configuration. |
| `<node_name>/fk_compute` | [Fk](https://github.com/Lab-CORO/curobo_msgs/blob/main/srv/Fk.srv) | Computes forward kinematics given joint positions. |
| `<node_name>/ik_batch_poses` | [Ik](https://github.com/Lab-CORO/curobo_msgs/blob/main/srv/Ik.srv) | Computes inverse kinematics for multiple target poses. |

## Parameters

| Name |  Type | Default| Description |
|---|---|---|---|
| `max_attempts`| `Int`|`1` |
| `timeout`| `Int`| `5.0`|
| `time_dilation_factor`| `Float`| `0.5` |[0, 1]
| `voxel_size`| `Float`|  `0.05`
| `collision_activation_distance`| `Float`| `0.025` |   |
| `robot_config_file`| `String` | `curobo_doosan/src/m1013/m1013.yml`|
## Actions
| Name | Message/Service Type | Description |
|---|---|---|
|`curobo_gen_traj/send_trajectrory`| [curobo_msgs/action/SendTrajectory.action](https://github.com/Lab-CORO/curobo_msgs/blob/main/action/SendTrajectory.action) |
