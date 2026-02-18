# Object management

## 1. Manage obstacles and objects

The motion generation and the inverse kinematics features can use obstacles and objects to avoid collisions during them execution. For both features, the objects are managed through a service interface started with the name of the feature and the name of the service. The robot spheres collisions are published on the topic `<node_name>/collision_spheres`. Currently, the robot pose is fixed at 0,0,0,0,0,0.

| Service Name | Service Type | Description | Callback Function |
|-------------|-------------|-------------|------------------|
| `<node_name>/add_object` | [`AddObject`](https://github.com/Lab-CORO/curobo_msgs/blob/main/srv/AddObject.srv) | Adds a new object to the scene, allowing collision checking with the environment. | `callback_add_object` |
| `<node_name>/remove_object` | [`RemoveObject`](https://github.com/Lab-CORO/curobo_msgs/blob/main/srv/RemoveObject.srv) | Removes a specific object from the scene. | `callback_remove_object` |
| `<node_name>/get_obstacles` | `Trigger` | Retrieves a list of all obstacles currently present in the scene. | `callback_get_obstacles` |
| `<node_name>/remove_all_objects` | `Trigger` | Clears all objects from the scene. | `callback_remove_all_objects` |
| `<node_name>/get_voxel_grid` | [`GetVoxelGrid`](https://github.com/Lab-CORO/curobo_msgs/blob/main/srv/GetVoxelGrid.srv) | Retrieves a voxelized representation of the environment for collision checking. | `callback_get_voxel_grid` |
| `<node_name>/get_collision_distance` | [`GetCollisionDistance`](https://github.com/Lab-CORO/curobo_msgs/blob/main/srv/GetCollisionDistance.srv) | Computes the distance between objects and potential collisions in the scene. | `callback_get_collision_distance` |

| Topic Name | Message Type | Description |
|-------------|-------------|-------------|
| `<node_name>/publish_collision_spheres` | `MarkerArray` | Publishes the collision spheres used for collision avoidance. |
