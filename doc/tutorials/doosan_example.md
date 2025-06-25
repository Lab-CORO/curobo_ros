## 🚀 Example with Doosan M1013

### 1. Motion Generation

The `curobo_gen_traj` node handles motion planning. You can launch it with:

```bash
ros2 launch curobo_ros gen_traj.launch.py
```

> ⚠️ On first launch, you may encounter the “missing symbol: ucm\_set\_global\_opts” error. The workaround is in [Troubleshooting](../troubleshooting.md#2-missing-symbol-ucm_set_global_opts), though a permanent fix is currently pending.

Once the node is running, interact with it using the `/curobo_gen_traj/generate_trajectory` ROS 2 service. You send a `geometry_msgs/Pose`; the node returns success or error. If successful, the trajectory is published to the `/trajectory` topic, which you can visualize in RViz using the **trajectory\_preview** plugin.

#### ⚙️ 2. Adjustable Parameters

- Some parameters can be changed:
  - Some can be changed online 
    * `max_attempts`: Maximum planning retries.
    * `timeout`: Time limit for trajectory planning.
    *  `time_dilation_factor`: The velocity of the robot. This percentage [0 , 1] is base on the joint velocity specified in the urdf/usd file.
  - Others have to be changed and the configuration should be update:
    * `voxel_size`: Collision grid cell resolution.
    * `collision_activation_distance`: Distance threshold to activate collision checking.
  
To execute the tragectory generated, an action server is available. To execute a trajectory, it have to be generated first with the service `/curobo_gen_traj/generate_trajectory`. The action send the trajector to the robot and wait for the end with a feedback message, the percentage of completion [0 , 1]. This execution depend n the implementation of your robot strategie. Currently, only the doosan m1013 strategie have been implemented. In next mounths, the Ur5e should be implemented.   

---


## ▶️ Up Next

* [Adding collision objects](adding_collision_objects.md)
* [Inverse & forward kinematics services](ik_fk_services.md)



