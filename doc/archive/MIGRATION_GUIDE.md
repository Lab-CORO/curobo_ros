# Guide de Migration vers l'Architecture Strategy Pattern

Ce guide vous aide à migrer votre code existant vers la nouvelle architecture unifiée de planification.

## 📋 Vue d'ensemble

**Avant:** Nodes séparés pour chaque type de planification
- `generate_trajectory.py` → Classic motion generation
- `mpc.py` → MPC control

**Après:** Architecture unifiée avec Strategy Pattern
- `unified_planner_node.py` → Support de tous les planners
- `planners/` → Modules réutilisables

## 🔄 Migration étape par étape

### Étape 1: Code existant (generate_trajectory.py)

```python
# ANCIEN CODE
class CuRoboTrajectoryMaker(Node):
    def generate_trajectrory_callback(self, request, response):
        # Get state
        start_state = JointState.from_position(...)

        # Get goal
        goal_pose = Pose.from_list([...])

        # Plan
        result = self.motion_gen.plan_single(
            start_state,
            goal_pose,
            MotionGenPlanConfig(...)
        )

        # Get trajectory
        traj = result.get_interpolated_plan()

        # Execute
        self.robot_context.set_command(
            traj.joint_names,
            traj.velocity.tolist(),
            traj.acceleration.tolist(),
            traj.position.tolist()
        )

        return response
```

### Étape 2: Nouveau code avec Strategy Pattern

```python
# NOUVEAU CODE
from curobo_ros.planners import PlannerFactory

class CuRoboTrajectoryMaker(Node):
    def __init__(self):
        # ...
        # Créer le planner
        self.planner = PlannerFactory.create_planner(
            'classic',
            self,
            self.config_wrapper
        )
        self.planner.set_motion_gen(self.motion_gen)

    def generate_trajectrory_callback(self, request, response):
        # Get state
        start_state = JointState.from_position(...)

        # Get goal
        goal_pose = Pose.from_list([...])

        # Config
        config = {
            'max_attempts': self.get_parameter('max_attempts').value,
            'timeout': self.get_parameter('timeout').value,
            'time_dilation_factor': self.get_parameter('time_dilation_factor').value,
        }

        # Plan
        result = self.planner.plan(start_state, goal_pose, config)

        # Execute
        if result.success:
            self.planner.execute(self.robot_context, goal_handle)

        response.success = result.success
        response.message = result.message
        return response
```

## 🎯 Cas d'usage spécifiques

### Cas 1: Garder les nodes séparés mais utiliser les planners

Vous pouvez garder vos nodes existants et juste utiliser les planners comme modules :

```python
# Dans generate_trajectory.py
from curobo_ros.planners import ClassicPlanner

class CuRoboTrajectoryMaker(Node):
    def __init__(self):
        # ... existing code ...

        # Ajouter le planner
        self.planner = ClassicPlanner(self, self.config_wrapper)
        self.planner.set_motion_gen(self.motion_gen)

    def generate_trajectrory_callback(self, request, response):
        # Utiliser le planner au lieu de motion_gen directement
        result = self.planner.plan(start_state, goal_pose, config)

        if result.success:
            # Option 1: Exécution via planner
            self.planner.execute(self.robot_context, goal_handle)

            # Option 2: Exécution manuelle (ancien code)
            traj = result.trajectory
            self.robot_context.set_command(...)
```

### Cas 2: Migrer complètement vers le node unifié

Remplacer les deux nodes par le node unifié :

**Avant:**
```bash
# Deux nodes séparés
ros2 run curobo_ros curobo_gen_traj
ros2 run curobo_ros curobo_mpc
```

**Après:**
```bash
# Un seul node, deux modes
ros2 run curobo_ros unified_planner --ros-args -p planner_type:=classic
ros2 run curobo_ros unified_planner --ros-args -p planner_type:=mpc
```

### Cas 3: Supporter plusieurs planners dans un node existant

```python
from curobo_ros.planners import PlannerManager

class MyNode(Node):
    def __init__(self):
        # ...
        self.planner_manager = PlannerManager(self, config_wrapper)

        # Declare parameter
        self.declare_parameter('planner_type', 'classic')

        # Set initial planner
        planner_type = self.get_parameter('planner_type').value
        self.planner_manager.set_current_planner(planner_type)

        # Service pour changer de planner
        self.create_service(
            Trigger,
            'set_planner',
            self.set_planner_callback
        )

    def set_planner_callback(self, request, response):
        planner_type = self.get_parameter('planner_type').value
        self.planner_manager.set_current_planner(planner_type)
        response.success = True
        return response

    def my_callback(self, request, response):
        # Utiliser le planner actuel
        planner = self.planner_manager.get_current_planner()
        result = planner.plan(start, goal, config)
        # ...
```

## 🔧 Modifications requises

### 1. Mise à jour de setup.py

Ajouter le nouveau node dans `setup.py` :

```python
entry_points={
    'console_scripts': [
        'curobo_gen_traj = curobo_ros.core.generate_trajectory:main',
        'curobo_mpc = curobo_ros.core.mpc:main',
        # NOUVEAU
        'unified_planner = curobo_ros.core.unified_planner_node:main',
    ],
},
```

### 2. Mise à jour des launch files

**Ancien:**
```python
Node(
    package='curobo_ros',
    executable='curobo_gen_traj',
    # ...
)
```

**Nouveau:**
```python
Node(
    package='curobo_ros',
    executable='unified_planner',
    parameters=[{
        'planner_type': 'classic',  # ou 'mpc'
    }],
    # ...
)
```

### 3. Mise à jour des appels de service

Les noms de service restent compatibles si vous gardez les nodes séparés.

Pour le node unifié :

```bash
# Ancien
ros2 service call /curobo_gen_traj/generate_trajectory ...

# Nouveau
ros2 service call /unified_planner/generate_trajectory ...
```

## ✅ Checklist de migration

- [ ] Créer les fichiers dans `curobo_ros/planners/`
- [ ] Tester les planners individuellement
- [ ] Décider : migrer complètement ou migration partielle ?
- [ ] Mettre à jour `setup.py` si nécessaire
- [ ] Mettre à jour les launch files
- [ ] Mettre à jour les scripts clients
- [ ] Tester avec vos configurations robot
- [ ] Documenter les changements pour votre équipe

## 🎓 Recommandations

### Pour de nouveaux projets
→ **Utiliser directement l'architecture Strategy Pattern**

### Pour des projets existants en production
→ **Migration progressive:**
1. Ajouter les planners comme modules
2. Tester en parallèle avec l'ancien code
3. Migrer progressivement les fonctionnalités
4. Remplacer complètement une fois validé

### Pour prototypage rapide
→ **Utiliser PlannerFactory directement**

```python
planner = PlannerFactory.create_planner('classic', node, config)
result = planner.plan(start, goal, config)
```

## 🐛 Dépannage

### Erreur: "No planner selected"
→ Vérifier que vous avez appelé `set_current_planner()` ou créé un planner

### Erreur: "MotionGen not initialized"
→ Appeler `planner.set_motion_gen(self.motion_gen)` après warmup

### Erreur: "MPC solver not initialized"
→ Appeler `planner.set_mpc_solver(self.mpc)` après initialisation

### Erreur: "Unknown planner type"
→ Vérifier les noms disponibles avec `PlannerFactory.get_available_planners()`

## 📞 Support

Pour questions ou problèmes :
1. Consulter le README dans `curobo_ros/planners/`
2. Vérifier les exemples dans `examples/planner_usage_example.py`
3. Ouvrir une issue sur le repo

## 🚀 Prochaines étapes

Après migration :
- [ ] Implémenter BatchPlanner
- [ ] Implémenter ConstrainedPlanner
- [ ] Ajouter des métriques de performance
- [ ] Créer des tests unitaires
- [ ] Optimiser les performances
