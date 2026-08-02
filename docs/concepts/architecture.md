# Package Architecture

`curobo_ros` is organized around one node, `unified_planner`, that composes three independent strategy families:

- **Planners** — *how* a motion is computed: open-loop trajectory planning or closed-loop reactive control.
- **Robot control strategies** — *where* joint commands go: an emulator, a velocity-streamed real robot, or a position-streamed real robot.
- **Camera strategies** — *what* the collision world perceives: depth cameras feeding a GPU voxel map.

Each family can be switched independently, at runtime, without touching the others. Everything shares a single GPU context (robot model, scene, collision caches) so that all solvers see the same world.

## System overview

```{mermaid}
graph TB
    subgraph Clients
        RVIZ[RViz panels<br/>curobo_rviz]
        CLI[ros2 CLI / user nodes]
    end

    subgraph UnifiedPlannerNode["unified_planner node"]
        SRV[Services & Action<br/>generate_trajectory, execute_trajectory, ...]
        PM[PlannerManager<br/>+ PlannerFactory]
        CW[ConfigWrapperMotion<br/>shared context: robot, Scene, caches]
        RC[RobotContext<br/>control strategies]
        CAM[CameraSystemManager<br/>depth cameras -> Mapper]
    end

    subgraph GPU["cuRobo v2 (GPU)"]
        MP[MotionPlanner]
        MPC[ModelPredictiveControl]
        IK[InverseKinematics]
        MAP[Mapper / ESDF]
    end

    ROBOT[Robot driver or emulator]

    RVIZ --> SRV
    CLI --> SRV
    SRV --> PM
    PM --> CW
    CW --> MP
    CW --> MPC
    CW --> IK
    CAM --> MAP
    MAP --> CW
    PM --> RC
    RC --> ROBOT
```

## Planner hierarchy

All planners implement the abstract `TrajectoryPlanner` interface (`curobo_ros/planners/trajectory_planner.py`) and declare an `ExecutionMode`: `OPEN_LOOP` (plan once, then stream) or `CLOSED_LOOP` (re-plan continuously while executing).

```{mermaid}
classDiagram
    class TrajectoryPlanner {
        <<abstract>>
        +plan(request) PlannerResult
        +execute(goal_handle)
        +get_execution_mode() ExecutionMode
    }
    class SinglePlanner {
        <<open-loop base>>
        shared MotionPlanner instance
    }
    class ReactiveController {
        <<closed-loop base>>
        servo loop, live goal, perception refresh
        +build_solver()
        +setup(state, goal)
        +step(state)
        +apply_live_goal(pose)
        +is_converged()
    }
    TrajectoryPlanner <|-- SinglePlanner
    TrajectoryPlanner <|-- ReactiveController
    SinglePlanner <|-- ClassicPlanner
    SinglePlanner <|-- MultiPointPlanner
    SinglePlanner <|-- JointSpacePlanner
    ReactiveController <|-- MPCController
    ReactiveController <|-- RetargetController
```

- **Open-loop** (`SinglePlanner` children) share one class-level cuRobo `MotionPlanner`: `ClassicPlanner` (single Cartesian goal), `MultiPointPlanner` (waypoint sequence), `JointSpacePlanner` (joint-space goal).
- **Closed-loop** (`ReactiveController` children) each wrap a cuRobo reactive solver and inherit the whole servo loop from the base class: `MPCController` (Model Predictive Control) and `RetargetController` (IK-based teleoperation follower).

`PlannerFactory` (`curobo_ros/planners/planner_factory.py`) holds the catalog mapping `SetPlanner` enum IDs to classes; `PlannerManager` caches instances and performs runtime switching. Adding a planner = subclass one of the two bases + one catalog entry. See [Unified Planner](unified-planner.md).

## Robot control strategy hierarchy

The robot side has two orthogonal axes (see [Tutorial 4](../tutorials/04-robot-execution.md)):

- **Which robot** — the `robot` parameter selects a descriptor `robots/<name>.yaml` (URDF, cuRobo config, topics).
- **How to command it** — the `control_strategy` parameter selects a `JointCommandStrategy` implementation.

```{mermaid}
classDiagram
    class RobotContext {
        strategy switching, lock order, buffer epochs
        +set_command(trajectory)
        +get_robot_state()
    }
    class JointCommandStrategy {
        <<abstract>>
        +send_trajectory()
        +get_progression()
        +stop_robot()
    }
    RobotContext o-- JointCommandStrategy
    JointCommandStrategy <|-- EmulatorStrategy
    JointCommandStrategy <|-- JointSpeedStrategy
    JointCommandStrategy <|-- JointPoseStrategy
    class GhostStrategy {
        always-on RViz preview
        publishes /trajectory
    }
```

- `EmulatorStrategy` — simulates the robot, publishes `/emulator/joint_states`.
- `JointSpeedStrategy` — streams `JointTrajectory` with velocities to a real robot bridge (used for the Doosan M1013 via the `leeloo` topics), with a hard acceleration clamp and real velocity feedback.
- `JointPoseStrategy` — streams positions only.
- `GhostStrategy` — not switchable; always publishes the planned trajectory on `/trajectory` for the RViz preview robot (namespace `preview/`).

`RobotContext` (`curobo_ros/robot/robot_context.py`) enforces a strict lock order (`gpu_lock > strategy_lock > buffer_lock`) and uses buffer epochs to refuse superseded trajectories.

## Configuration and managers

`ConfigWrapperMotion` composes five managers that own startup and the shared state; the deep dive is in [Manager Architecture](manager-architecture.md):

1. `ConfigManager` — robot descriptor, cuRobo YAML resolution, world file.
2. `RobotModelManager` — cuRobo `Kinematics`, collision spheres, per-link collision toggling.
3. `ObstacleManager` — the single `Scene`, obstacle add/remove, collision caches, voxel grid, `Mapper` perception.
4. `CameraSystemManager` — camera strategies from `cameras.yaml`.
5. `RosServiceManager` — obstacle/introspection services and visualization publishers.

## Key invariants

- **One GPU context.** Every solver (planning, MPC, IK) is built from the same shared context — same robot model, same scene, same caches. There is no separate world for MPC.
- **One live CUDA graph.** With `use_cuda_graph: true`, the node guarantees at most one captured CUDA graph across solvers; switching planners releases and re-captures as needed.
- **Push-based perception.** Depth frames are integrated into the `Mapper` as they arrive (under a non-blocking GPU lock); solvers read the resulting ESDF. See [Tutorial 7](../tutorials/07-pointcloud-detection.md).

## Source layout

```
curobo_ros/
├── core/           # node, managers, IK/FK/attachment services, segmentation
├── planners/       # TrajectoryPlanner hierarchy + factory/manager
├── robot/          # RobotContext, control strategies, robot descriptors
├── cameras/        # camera strategies (depth_camera)
├── launch/         # gen_traj.launch.py and helpers
├── robots/         # robot descriptors (doosan_m1013.yaml, emulator.yaml)
└── config/         # cameras.yaml, floor_world.yml, test specs
```

## Related pages

- [Unified Planner](unified-planner.md) — planner catalog, switching, warmup costs
- [Manager Architecture](manager-architecture.md) — the five managers in detail
- [MPC Implementation](mpc-implementation.md) — the reactive control loop
- [ROS Interfaces](ros-interfaces.md) — every service, action, and topic
