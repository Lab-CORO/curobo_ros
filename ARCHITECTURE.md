# Architecture Strategy Pattern - Vue d'ensemble

## 📐 Diagramme de l'architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    ROS2 Node (Client)                           │
│                                                                 │
│  ros2 run curobo_ros unified_planner                           │
│       --ros-args -p planner_type:=classic                      │
└────────────────────┬────────────────────────────────────────────┘
                     │
                     │ Services / Actions
                     ▼
┌─────────────────────────────────────────────────────────────────┐
│               UnifiedPlannerNode                                │
│                                                                 │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │              PlannerManager                              │  │
│  │                                                          │  │
│  │  ┌────────────────┐  ┌────────────────┐                │  │
│  │  │ Classic Cache  │  │  MPC Cache     │  ...           │  │
│  │  └────────────────┘  └────────────────┘                │  │
│  │          │                    │                         │  │
│  │          └────────┬───────────┘                         │  │
│  │                   │                                     │  │
│  │                   ▼                                     │  │
│  │         Current Planner                                │  │
│  └──────────────────────────────────────────────────────────┘  │
│                     │                                          │
│                     │                                          │
└─────────────────────┼──────────────────────────────────────────┘
                      │
                      ▼
        ┌─────────────────────────────┐
        │   TrajectoryPlanner (ABC)   │
        │                             │
        │  + plan()                   │
        │  + execute()                │
        │  + get_execution_mode()     │
        └─────────────────────────────┘
                      △
                      │
        ┌─────────────┴─────────────┐
        │                           │
┌───────┴────────┐         ┌────────┴────────┐
│ ClassicPlanner │         │   MPCPlanner    │
│                │         │                 │
│ Mode: OPEN_LOOP│         │ Mode: CLOSED_LOOP│
│                │         │                 │
│ ┌────────────┐ │         │ ┌─────────────┐ │
│ │ MotionGen  │ │         │ │ MpcSolver   │ │
│ └────────────┘ │         │ └─────────────┘ │
│                │         │                 │
│ plan():        │         │ plan():         │
│  - Generate    │         │  - Setup goal   │
│    full traj   │         │    buffer       │
│                │         │                 │
│ execute():     │         │ execute():      │
│  - Send all    │         │  - Loop:        │
│    waypoints   │         │    * MPC step   │
│  - Monitor     │         │    * Send cmd   │
│    progress    │         │    * Check goal │
└────────────────┘         └─────────────────┘
        │                           │
        └─────────────┬─────────────┘
                      │
                      ▼
        ┌─────────────────────────────┐
        │      RobotContext           │
        │                             │
        │  + set_command()            │
        │  + send_trajectory()        │
        │  + get_progression()        │
        │  + stop_robot()             │
        └─────────────────────────────┘
                      │
                      ▼
        ┌─────────────────────────────┐
        │   Robot Strategy (Pattern)  │
        │                             │
        │  - DoosanStrategy           │
        │  - EmulatorStrategy         │
        │  - GhostStrategy            │
        └─────────────────────────────┘
```

## 🔄 Flux d'exécution

### Flux Open-Loop (ClassicPlanner)

```
User Request
    │
    ▼
[Plan Phase]
    │
    ├─→ Get current state
    ├─→ MotionGen.plan_single()
    ├─→ Generate full trajectory
    │   (position, velocity, acceleration)
    └─→ Store trajectory
    │
    ▼
[Execute Phase]
    │
    ├─→ Send ALL waypoints to RobotContext
    ├─→ RobotContext → Robot Strategy
    └─→ Monitor progression (0% → 100%)
    │
    ▼
Done
```

### Flux Closed-Loop (MPCPlanner)

```
User Request
    │
    ▼
[Setup Phase]
    │
    ├─→ Get current state
    ├─→ Setup MPC goal buffer
    └─→ Initialize MPC
    │
    ▼
[Execution Loop]
    │
    ├─→ Get current state
    │   │
    │   ▼
    │  [MPC Optimization]
    │   │
    │   ├─→ Predict future states
    │   ├─→ Evaluate cost function
    │   ├─→ Optimize next action
    │   └─→ Return optimal action
    │   │
    │   ▼
    ├─→ Send single action to Robot
    ├─→ Update current state
    ├─→ Check convergence
    │   │
    │   ├─ Not converged → Loop back
    │   └─ Converged → Exit
    │
    ▼
Done
```

## 🎯 Séparation des responsabilités

| Composant | Responsabilité | Exemples |
|-----------|----------------|----------|
| **TrajectoryPlanner** | Interface abstraite | Définit plan() et execute() |
| **ClassicPlanner** | Algorithme open-loop | MotionGen + exécution complète |
| **MPCPlanner** | Algorithme closed-loop | MPC + boucle temps réel |
| **PlannerFactory** | Création d'instances | create_planner('classic') |
| **PlannerManager** | Gestion multi-planners | Switching dynamique |
| **RobotContext** | Communication robot | Envoi commandes, monitoring |
| **ConfigWrapper** | Configuration cuRobo | World config, robot config |

## 📊 Comparaison des approches

### Ancienne architecture (≤ Maintenant)

```
generate_trajectory.py          mpc.py
        │                          │
        │                          │
        ▼                          ▼
   MotionGen                   MpcSolver
        │                          │
        └──────────┬───────────────┘
                   │
                   ▼
            RobotContext
```

**Problèmes:**
- ❌ Code dupliqué (deux nodes similaires)
- ❌ Difficile d'ajouter de nouveaux planners
- ❌ Impossible de switcher dynamiquement
- ❌ Logique métier mélangée avec ROS

### Nouvelle architecture (Strategy Pattern)

```
   unified_planner_node.py
            │
            ▼
      PlannerManager
            │
    ┌───────┴───────┐
    │               │
ClassicPlanner  MPCPlanner
    │               │
    └───────┬───────┘
            │
            ▼
      RobotContext
```

**Avantages:**
- ✅ Code réutilisable et modulaire
- ✅ Facile d'ajouter BatchPlanner, ConstrainedPlanner
- ✅ Switching dynamique via paramètres
- ✅ Séparation claire des responsabilités
- ✅ Testable indépendamment

## 🔌 Points d'extension

### 1. Ajouter un nouveau planner

```python
class BatchPlanner(TrajectoryPlanner):
    def _get_execution_mode(self):
        return ExecutionMode.OPEN_LOOP

    def plan(self, start, goal, config):
        # Générer N trajectoires
        trajectories = []
        for i in range(config['num_trajectories']):
            result = self.motion_gen.plan_single(...)
            trajectories.append(result)

        # Sélectionner la meilleure
        best = select_best(trajectories)
        return PlannerResult(success=True, trajectory=best)

    def execute(self, robot_context, goal_handle):
        # Exécution standard
        robot_context.set_command(...)

# Enregistrer
PlannerFactory.register_planner('batch', BatchPlanner)
```

### 2. Ajouter des métriques

```python
class MetricsPlanner(TrajectoryPlanner):
    def __init__(self, node, config_wrapper):
        super().__init__(node, config_wrapper)
        self.metrics = {
            'planning_time': [],
            'execution_time': [],
            'success_rate': 0
        }

    def plan(self, start, goal, config):
        t0 = time.time()
        result = super().plan(start, goal, config)
        self.metrics['planning_time'].append(time.time() - t0)
        return result
```

### 3. Ajouter de la validation

```python
class ValidatedPlanner(TrajectoryPlanner):
    def plan(self, start, goal, config):
        # Validation pré-planning
        if not self._validate_goal(goal):
            return PlannerResult(success=False, message="Invalid goal")

        # Plan
        result = super().plan(start, goal, config)

        # Validation post-planning
        if result.success and not self._validate_trajectory(result.trajectory):
            return PlannerResult(success=False, message="Unsafe trajectory")

        return result
```

## 🎨 Patterns utilisés

### 1. Strategy Pattern
**But:** Encapsuler des algorithmes interchangeables

```python
# Client code doesn't know which algorithm is used
planner = get_current_planner()  # Could be Classic or MPC
result = planner.plan(start, goal, config)  # Same interface
```

### 2. Factory Pattern
**But:** Création centralisée d'objets

```python
# Instead of: planner = ClassicPlanner(...)
# Use factory:
planner = PlannerFactory.create_planner('classic', node, config)
```

### 3. Template Method Pattern
**But:** Définir le squelette d'un algorithme

```python
class TrajectoryPlanner(ABC):
    def plan_and_execute(self, start, goal, robot_context):
        # Template method
        result = self.plan(start, goal, config)  # Implemented by subclass
        if result.success:
            self.execute(robot_context)  # Implemented by subclass
```

## 🚀 Extensions futures

1. **HybridPlanner:** Classic + MPC
   - Plan global avec Classic
   - Suivi local avec MPC

2. **LearningPlanner:** Apprentissage par renforcement
   - Apprend de trajectoires précédentes
   - Améliore les performances

3. **MultiRobotPlanner:** Coordination multi-robots
   - Planification collaborative
   - Évitement inter-robots

4. **AdaptivePlanner:** Sélection automatique
   - Choisit le meilleur planner selon contexte
   - Environnement statique → Classic
   - Environnement dynamique → MPC
