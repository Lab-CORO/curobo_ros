# Package architecture

## Overview
 
curobo_ros uses a modular, pattern-based architecture that separates concerns between planning, robot control, and visualization. The architecture supports:
 
- **Multiple planning strategies** (Classic, MPC, Batch, Constrained)
- **Multiple robot control strategies** (Real robot, Emulator, Ghost)
- **Dynamic switching** between strategies at runtime
- **Extensibility** for custom planners and robot controllers
 
---
 
## Unified Planner Architecture
 
The unified planner implements the **Strategy Pattern** to support multiple planning algorithms:
 
```mermaid
classDiagram
    class TrajectoryPlanner {
        <<abstract>>
        +plan(start, goal, config) PlannerResult
        +execute(robot_context, goal_handle) bool
        +get_execution_mode() ExecutionMode
        +get_planner_name() str
    }
 
    class ClassicPlanner {
        +ExecutionMode: OPEN_LOOP
        +plan() PlannerResult
        +execute() bool
    }
 
    class MPCPlanner {
        +ExecutionMode: CLOSED_LOOP
        +plan() PlannerResult
        +execute() bool
        -convergence_threshold: float
        -max_iterations: int
    }
 
    class BatchPlanner {
        +ExecutionMode: OPEN_LOOP
        +plan() PlannerResult
        +execute() bool
        -num_trajectories: int
    }
 
    class ConstrainedPlanner {
        +ExecutionMode: OPEN_LOOP
        +plan() PlannerResult
        +execute() bool
        -custom_constraints: dict
    }
 
    class PlannerFactory {
        <<factory>>
        +create_planner(type, node, config) TrajectoryPlanner
        +register_planner(name, class)
        +list_planners() list
    }
 
    class PlannerManager {
        -current_planner: TrajectoryPlanner
        -available_planners: dict
        +set_current_planner(name) bool
        +get_current_planner() TrajectoryPlanner
        +list_available_planners() list
    }
 
    class UnifiedPlannerNode {
        -planner_manager: PlannerManager
        +generate_trajectory_callback()
        +execute_trajectory_callback()
        +set_planner_callback()
    }
 
    TrajectoryPlanner <|-- ClassicPlanner
    TrajectoryPlanner <|-- MPCPlanner
    TrajectoryPlanner <|-- BatchPlanner
    TrajectoryPlanner <|-- ConstrainedPlanner
    PlannerFactory ..> TrajectoryPlanner : creates
    PlannerManager o-- TrajectoryPlanner : manages
    UnifiedPlannerNode --> PlannerManager : uses
```
 
**Key Features:**
- **Strategy Pattern**: Easy to add new planners
- **Factory Pattern**: Centralized planner creation
- **Manager Pattern**: Runtime planner switching
- **Type-safe**: Enum-based planner selection
 
For details, see [Unified Planner Documentation](unified_planner.md).
 
---
 
## Robot Control Strategy Architecture
 
Robot control also uses the Strategy Pattern for different robot types:
 
```mermaid
classDiagram
    class JointCommandStrategy {
        <<abstract>>
        +send_trajectory()
        +get_joint_pose() array
        +get_joint_name() list
        +stop_robot()
        +get_progression() float
    }
 
    class DoosanControl {
        +send_trajectory()
        +get_joint_pose()
        -topic: /leeloo/execute_trajectory
    }
 
    class EmulatorStrategy {
        +send_trajectory()
        +get_joint_pose()
        -topic: /joint_states
        -simulation_thread: Thread
    }
 
    class GhostStrategy {
        +send_trajectory()
        +get_joint_pose()
        -topic: /trajectory
        -instant_display: bool
    }
 
    class RobotContext {
        -strategy: JointCommandStrategy
        -strategy_lock: Lock
        +set_command(trajectory)
        +send_trajectory()
        +select_strategy(robot_type)
        +get_robot_state() RobotState
    }
 
    JointCommandStrategy <|-- DoosanControl
    JointCommandStrategy <|-- EmulatorStrategy
    JointCommandStrategy <|-- GhostStrategy
    RobotContext o-- JointCommandStrategy : uses
```
 
**Key Features:**
- **Strategy Pattern**: Easy to add new robots
- **Thread-safe**: Lock-protected strategy switching
- **Runtime switching**: Change robot without restart
- **Ghost overlay**: Always-on trajectory preview
 
---
 
## Overall System Architecture
 
```mermaid
graph TB
    subgraph "Planning Layer"
        UP[UnifiedPlannerNode]
        PM[PlannerManager]
        CP[ClassicPlanner]
        MP[MPCPlanner]
    end
 
    subgraph "Control Layer"
        RC[RobotContext]
        DS[DoosanControl]
        ES[EmulatorStrategy]
        GS[GhostStrategy]
    end
 
    subgraph "Configuration Layer"
        CW[ConfigWrapper]
        CWM[ConfigWrapperMotion]
        CWI[ConfigWrapperIK]
    end
 
    subgraph "Visualization Layer"
        MPub[MarkerPublisher]
        RViz[RViz2]
    end
 
    UP --> PM
    PM --> CP
    PM --> MP
    UP --> RC
    RC --> DS
    RC --> ES
    RC --> GS
    UP --> CWM
    CWM --> CW
    CWI --> CW
    UP --> MPub
    MPub --> RViz
    DS --> RViz
    ES --> RViz
    GS --> RViz
```
 
---
 
## Class diagramm