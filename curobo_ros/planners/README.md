# Trajectory Planner Architecture

Architecture Strategy Pattern pour la génération de trajectoires avec cuRobo.

## 📋 Vue d'ensemble

Cette architecture unifie différentes méthodes de planification de trajectoire :

| Planner | Mode | Description | Cas d'usage |
|---------|------|-------------|-------------|
| **ClassicPlanner** | Open-loop | Génération complète puis exécution | Navigation simple, trajectoires pré-calculées |
| **MPCPlanner** | Closed-loop | Recalcul temps réel à chaque pas | Contrôle réactif, environnements dynamiques |
| **BatchPlanner** | Open-loop | Génération de multiples trajectoires | Optimisation de tâches multiples |
| **ConstrainedPlanner** | Open-loop | Planification avec contraintes custom | Contraintes spécifiques (orientation, vitesse) |

## 🏗️ Architecture

```
TrajectoryPlanner (ABC)
├── plan(start, goal, config) → PlannerResult
├── execute(robot_context, goal_handle) → bool
└── get_execution_mode() → ExecutionMode

ClassicPlanner
├── Mode: OPEN_LOOP
├── Algorithme: MotionGen
└── Execution: Trajectoire complète d'un coup

MPCPlanner
├── Mode: CLOSED_LOOP
├── Algorithme: MPC Solver
└── Execution: Boucle itérative temps réel
```

## 🚀 Utilisation

### 1. Utilisation du node unifié

```bash
# Lancer le node unifié
ros2 run curobo_ros unified_planner

# Lister les planners disponibles
ros2 service call /unified_planner/list_planners std_srvs/srv/Trigger

# Changer de planner (avec enum type-safe)
ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 0}"  # CLASSIC
ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 1}"  # MPC
ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 2}"  # BATCH
ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 3}"  # CONSTRAINED

# Générer une trajectoire
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration "..."

# Exécuter
ros2 action send_goal /unified_planner/execute_trajectory curobo_msgs/action/SendTrajectory "{}"
```

### 2. Utilisation programmatique

```python
from curobo_ros.planners import PlannerFactory, PlannerManager

# Créer un planner directement
planner = PlannerFactory.create_planner('classic', node, config_wrapper)

# Ou utiliser le manager pour gérer plusieurs planners
manager = PlannerManager(node, config_wrapper)
manager.set_current_planner('classic')

# Planifier
result = planner.plan(start_state, goal_pose, config)

if result.success:
    # Exécuter
    success = planner.execute(robot_context, goal_handle)
```

### 3. Créer un planner custom

```python
from curobo_ros.planners import TrajectoryPlanner, ExecutionMode, PlannerResult

class MyCustomPlanner(TrajectoryPlanner):
    def _get_execution_mode(self) -> ExecutionMode:
        return ExecutionMode.OPEN_LOOP

    def get_planner_name(self) -> str:
        return "My Custom Planner"

    def plan(self, start_state, goal_pose, config) -> PlannerResult:
        # Votre algorithme de planification
        trajectory = my_planning_algorithm(start_state, goal_pose)

        return PlannerResult(
            success=True,
            message="Planning succeeded",
            trajectory=trajectory
        )

    def execute(self, robot_context, goal_handle=None) -> bool:
        # Votre logique d'exécution
        robot_context.set_command(...)
        robot_context.send_trajectory()
        return True

# Enregistrer votre planner
from curobo_ros.planners import PlannerFactory
PlannerFactory.register_planner('custom', MyCustomPlanner)

# Utiliser
planner = PlannerFactory.create_planner('custom', node, config_wrapper)
```

## 📊 Comparaison des planners

### ClassicPlanner (Open-loop)

**Avantages:**
- ✅ Trajectoire complète optimisée
- ✅ Prévisible et reproductible
- ✅ Efficace pour environnements statiques

**Inconvénients:**
- ❌ Pas de réaction aux changements
- ❌ Nécessite replanification si perturbation

**Paramètres:**
```python
config = {
    'max_attempts': 1,
    'timeout': 5.0,
    'time_dilation_factor': 0.5
}
```

### MPCPlanner (Closed-loop)

**Avantages:**
- ✅ Réactif aux perturbations
- ✅ Adaptatif en temps réel
- ✅ Gère environnements dynamiques

**Inconvénients:**
- ❌ Plus coûteux en calcul
- ❌ Nécessite GPU pour temps réel
- ❌ Moins prévisible

**Paramètres:**
```python
config = {
    'convergence_threshold': 0.01,  # meters
    'max_iterations': 1000
}
```

## 🔧 Intégration dans un node existant

### Option 1: Remplacer generate_trajectory.py

```python
# Ancien code
result = self.motion_gen.plan_single(start, goal, config)
traj = result.get_interpolated_plan()
robot_context.set_command(traj.joint_names, ...)

# Nouveau code avec Strategy Pattern
planner = PlannerFactory.create_planner('classic', self, config_wrapper)
planner.set_motion_gen(self.motion_gen)
result = planner.plan(start, goal, config)
if result.success:
    planner.execute(robot_context, goal_handle)
```

### Option 2: Supporter plusieurs planners

```python
class MyNode(Node):
    def __init__(self):
        # ...
        self.planner_manager = PlannerManager(self, config_wrapper)
        self.planner_manager.set_current_planner('classic')

    def callback(self, request, response):
        planner = self.planner_manager.get_current_planner()
        result = planner.plan(start, goal, config)
        # ...
```

## 📦 Structure des fichiers

```
curobo_ros/planners/
├── __init__.py                  # Exports publics
├── trajectory_planner.py        # Classe abstraite
├── classic_planner.py           # Implémentation Classic
├── mpc_planner.py               # Implémentation MPC
├── planner_factory.py           # Factory et Manager
└── README.md                    # Cette documentation
```

## 🎯 Design Patterns utilisés

1. **Strategy Pattern**: Encapsule différents algorithmes de planification
2. **Factory Pattern**: Création centralisée des planners
3. **Template Method**: Méthodes communes dans classe abstraite

## 🔮 Extensions futures

Planners à implémenter :

```python
# Batch planner
class BatchPlanner(TrajectoryPlanner):
    """Génère plusieurs trajectoires alternatives"""
    pass

# Constrained planner
class ConstrainedPlanner(TrajectoryPlanner):
    """Planification avec contraintes custom"""
    pass

# Hybrid planner
class HybridPlanner(TrajectoryPlanner):
    """Combine Classic + MPC"""
    pass
```

## 📚 Ressources

- [Design Patterns](https://refactoring.guru/design-patterns/strategy)
- [cuRobo Documentation](https://curobo.org/)
- [ROS2 Actions](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)
