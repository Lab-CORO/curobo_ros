# Unified Planner Architecture

The **Unified Planner** is the central node of curobo_ros. It exposes a single ROS 2 node (`unified_planner`) that supports multiple planning algorithms, kinematics services, and obstacle management through a unified interface.

---

## Overview

The unified planner uses the **Strategy Pattern** to encapsulate different trajectory planning algorithms, making it easy to switch between planning methods at runtime without modifying client code.

### Available Planners

| Planner | Enum ID | Mode | Description | Use Case |
|---------|---------|------|-------------|----------|
| **Classic** | 0 | Open-loop | Single-shot trajectory generation | Static environments, pre-computed paths |
| **MPC** | 1 | Closed-loop | Real-time replanning at each step | Dynamic environments, moving obstacles |
| **Multi-Point** | 4 | Open-loop | Waypoint-based trajectory generation | Pick-and-place, inspection paths |
| **Joint Space** | 5 | Open-loop | Planning in joint space | Direct joint configuration targets |

> **Note**: Constrained planning (hold orientation, lock axes) is available in Classic and Multi-Point planners via the `trajectory_constraints` field — it does not require a separate planner type.

---

## Architecture

### Node Structure

```
UnifiedPlannerNode  (/unified_planner)
│
├── Trajectory Planners (Strategy Pattern)
│   ├── ClassicPlanner       — MotionGen, open-loop
│   ├── MPCPlanner           — MpcSolver, closed-loop
│   ├── MultiPointPlanner    — MotionGen, waypoints
│   └── JointSpacePlanner    — MotionGen, joint targets
│
├── Kinematics Services (lazy warmup)
│   ├── IKServices           — IkSolver, shares obstacles
│   └── FKServices           — CudaRobotModel, geometry only
│
└── Shared Infrastructure
    ├── ObstacleManager      — single source of truth for world
    ├── RobotContext         — robot strategy (real/emulator/ghost)
    └── ConfigWrapperMotion  — robot config, camera, parameters
```

### Planner Class Hierarchy

```
TrajectoryPlanner (abstract)
│
├── SinglePlanner (shared MotionGen instance)
│   ├── ClassicPlanner
│   ├── MultiPointPlanner
│   └── JointSpacePlanner
│
└── MPCPlanner (independent MpcSolver)
```

**SinglePlanner** is an intermediate abstract class for all planners that use cuRobo's MotionGen. All `SinglePlanner` children share a **single** MotionGen instance — warmup is performed only once, and switching between them is instantaneous with no re-warmup cost.

**MPCPlanner** uses a separate `MpcSolver` instance. It can optionally share the world collision checker with MotionGen to avoid duplicating obstacle data in VRAM.

---

## Planner Details

### Classic Planner

Generates a complete collision-free trajectory from start to goal in one shot, then executes it open-loop.

**Strengths:** Predictable, reproducible, low GPU overhead during execution.
**Limitations:** No reaction to environment changes mid-execution.
**Best for:** Static, well-defined environments.

Supports optional trajectory constraints (hold orientation, lock axes) via the `trajectory_constraints` field.

---

### MPC Planner

Continuously replans in a closed-loop at high frequency using cuRobo's Model Predictive Control solver.

**Strengths:** Reactive to perturbations and dynamic obstacles, high-precision tracking.
**Limitations:** Sustained GPU usage, less predictable completion time.
**Best for:** Dynamic environments, moving targets, disturbance rejection.

Convergence is declared when the end-effector reaches within `convergence_threshold` (default 0.01m) of the target.

---

### Multi-Point Planner

Plans a trajectory through a sequence of Cartesian waypoints in a single optimized pass.

**Strengths:** Efficient multi-waypoint tasks without chaining multiple service calls.
**Limitations:** All waypoints must be reachable; failure at one invalidates the whole plan.
**Best for:** Pick-and-place sequences, inspection paths, structured workflows.

---

### Joint Space Planner

Plans a trajectory to a target joint configuration instead of a Cartesian pose.

**Strengths:** Precise joint-level control, avoids IK ambiguity.
**Best for:** Moving to a known home configuration, calibration poses.

---

## IK and FK Services

IK and FK are **services on the same node** as the trajectory planner. They are **not initialized at startup** — each solver is created only when the corresponding warmup service is called (lazy initialization).

### Design Rationale

| | IKServices | FKServices |
|---|---|---|
| Solver | `IkSolver` (cuRobo) | `CudaRobotModel` (cuRobo) |
| Obstacle dependency | **Yes** — shares `ObstacleManager` | **No** — geometry only |
| World updates | Receives obstacle changes automatically | Not needed |
| Warmup service | `warmup_ik` (with `batch_size`) | `warmup_fk` (with `batch_size`) |

Because IK uses collision avoidance, it shares the same obstacle world as the trajectory planners. When obstacles are added or removed, the IK solver is updated automatically — no manual synchronization needed.

FK is purely geometric (forward kinematics has no collision component), so it depends only on the robot's kinematic model.

### Warmup and Batch Size

The IK solver pre-allocates GPU memory for a fixed batch size. If you call `ik_batch` with a different number of poses than the warmup batch size, the solver reinitializes automatically (the first call will be slower). For best performance, warmup with the batch size you intend to use.

FK warmup primes the CUDA kernels with the expected batch size but does not require reinitialization when the batch size changes.

See [ROS Interfaces — Kinematics Services](ros-interfaces.md#kinematics-services) for full service specifications and CLI examples.

---

## Runtime Planner Switching

Planners can be switched at runtime without restarting the node:

```bash
# Switch to MPC
ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 1}"

# Switch back to Classic
ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 0}"

# Query available planners and their enum IDs
ros2 service call /unified_planner/get_planners curobo_msgs/srv/GetPlanners
```

Switching between Classic / Multi-Point / Joint Space is **instantaneous** (shared MotionGen). Switching to or from MPC requires its solver to be initialized (lazy warmup on first switch, ~5–15s).

---

## Execution Modes

**Open-loop** (Classic, Multi-Point, Joint Space):
- Full trajectory computed during `generate_trajectory`, then sent to robot via `execute_trajectory`
- Predictable and reproducible
- Requires replanning if environment changes mid-execution

**Closed-loop** (MPC):
- No pre-computed trajectory — the solver computes the next command at each control cycle
- Reactive to real-time changes
- Execution continues until convergence or cancellation via the `execute_trajectory` action

---

## Services and Actions Summary

| Service / Action | Type | Description |
|-----------------|------|-------------|
| `generate_trajectory` | `TrajectoryGeneration` | Plan with current planner |
| `execute_trajectory` (action) | `SendTrajectory` | Execute planned trajectory |
| `set_planner` | `SetPlanner` | Switch active planner |
| `get_planners` | `GetPlanners` | List available planners with enum IDs |
| `warmup_ik` | `WarmupIK` | Initialize IK solver |
| `warmup_fk` | `WarmupFK` | Initialize FK model |
| `ik` | `Ik` | Single IK query |
| `ik_batch` | `IkBatch` | Batch IK query |
| `fk` | `Fk` | Batch FK query |

---

## File Structure

```
curobo_ros/
├── core/
│   ├── unified_planner_node.py   # Main node
│   ├── ik_services.py            # IK lazy services
│   ├── fk_services.py            # FK lazy services
│   ├── config_wrapper_motion.py  # MotionGen config
│   └── obstacle_manager.py       # World state
│
└── planners/
    ├── trajectory_planner.py     # Abstract base class
    ├── single_planner.py         # Shared MotionGen base
    ├── classic_planner.py
    ├── mpc_planner.py
    ├── multi_point_planner.py
    ├── joint_space_planner.py
    └── planner_factory.py        # Factory and Manager
```

---

## Performance

| Planner | Planning time | GPU during execution | Notes |
|---------|--------------|---------------------|-------|
| Classic | 10–100ms | Idle | Best for static environments |
| Multi-Point | 20–200ms | Idle | Scales with number of waypoints |
| Joint Space | 10–100ms | Idle | Fastest when goal is in joint space |
| MPC | 1–10ms/iter | Sustained | Continuous replanning loop |

---

## Design Patterns

- **Strategy Pattern** — planning algorithms are interchangeable at runtime
- **Factory Pattern** — `PlannerFactory` centralizes planner creation and registration
- **Template Method** — `SinglePlanner` defines the execution skeleton; children implement only the planning logic
- **Lazy Initialization** — IK, FK, and MPC solvers are created only when first needed

---

## Related Documentation

- [ROS Interfaces](ros-interfaces.md) — complete service and action reference
- [Tutorial 1: First Trajectory](../tutorials/01-first-trajectory.md)
- [Tutorial 5: MPC Planner](../tutorials/05-mpc-planner.md)
- [Tutorial 6: IK/FK Services](../tutorials/06-ik-fk-services.md)
- [Manager Architecture](manager-architecture.md)
