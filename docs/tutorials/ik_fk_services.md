# Inverse and forward kinenatics

## 1. Batch Inverse Kinematics (IK)

These services compute inverse kinematics for multiple target poses while also utilizing object management services. The node of this feature is `curobo_ik`. As the motion generation some parameters can be change and a service is available to update the parameters.

| Service Name | Service Type | Description | Callback Function |
|-------------|-------------|-------------|------------------|
| `<node_name>/ik_batch_poses` | [`Ik`](https://github.com/Lab-CORO/curobo_msgs/blob/main/srv/Ik.srv) | Computes the inverse kinematics (IK) for multiple target poses. | `ik_callback` |

Batch IK relies on the same object management services used for motion generation, ensuring collision-aware calculations.

---

## 2. Forward Kinematics (FK) Services

Unlike motion generation and batch IK, forward kinematics does not use object management services.

| Service Name | Service Type | Description | Callback Function |
|-------------|-------------|-------------|------------------|
| `<node_name>/fk_compute` | [`Fk`](https://github.com/Lab-CORO/curobo_msgs/blob/main/srv/Fk.srv) | Computes the forward kinematics given a set of joint positions. | `fk_callback` |
