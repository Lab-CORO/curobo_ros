# Tests d'Intégration curobo_ros

Tests d'intégration complets pour valider tous les services, actions et topics ROS2 du package curobo_ros.

## 📁 Structure

```
tests/
├── integration/                    # Tests d'intégration ROS2
│   ├── test_fk_services.py        # Tests Forward Kinematics
│   ├── test_ik_services.py        # Tests Inverse Kinematics
│   ├── test_trajectory_generation.py  # Tests génération trajectoire (curobo_gen_traj)
│   ├── test_obstacle_management.py    # Tests gestion obstacles
│   ├── test_robot_segmentation.py     # Tests segmentation robot
│   ├── test_unified_planner.py        # Tests UnifiedPlannerNode
│   ├── test_mpc_planner.py            # Tests MPC planner
│   ├── test_planner_switching.py      # Tests changement de planner
│   └── test_full_pipeline.py          # Tests bout-en-bout
├── fixtures/                       # Données de test réutilisables
│   ├── test_poses.py              # Poses et joint states
│   └── test_robot_configs.py      # Configurations
└── launch/                         # Launch files pour tests
    └── integration_tests.launch.py

```

## 🚀 Exécution des Tests

### Prérequis

Les nœuds ROS2 doivent être lancés avant d'exécuter les tests.

#### Option 1: Tests avec nœud classique (curobo_gen_traj)

```bash
# Terminal 1: Lancer les nœuds
ros2 launch curobo_ros gen_traj.launch.py

# Terminal 2: Lancer le nœud IK
ros2 run curobo_ros curobo_ik

# Terminal 3: Lancer le nœud FK
ros2 run curobo_ros curobo_fk

# Terminal 4: Exécuter les tests
cd /home/user/curobo_ros
pytest tests/integration/test_fk_services.py -v
pytest tests/integration/test_ik_services.py -v
pytest tests/integration/test_trajectory_generation.py -v
pytest tests/integration/test_obstacle_management.py -v
```

#### Option 2: Tests avec nœud unifié (unified_planner)

```bash
# Terminal 1: Lancer le unified planner
ros2 run curobo_ros curobo_trajectory_planner

# Terminal 2: Lancer les nœuds IK/FK
ros2 run curobo_ros curobo_ik
ros2 run curobo_ros curobo_fk

# Terminal 3: Exécuter les tests
cd /home/user/curobo_ros
pytest tests/integration/test_unified_planner.py -v
pytest tests/integration/test_mpc_planner.py -v
pytest tests/integration/test_planner_switching.py -v
pytest tests/integration/test_full_pipeline.py -v
```

### Exécuter tous les tests

```bash
# Tous les tests d'intégration
pytest tests/integration/ -v

# Tests avec rapport de couverture
pytest tests/integration/ -v --cov=curobo_ros --cov-report=html

# Tests spécifiques avec marqueurs
pytest tests/integration/ -v -k "fk"
pytest tests/integration/ -v -k "mpc"
pytest tests/integration/ -v -k "obstacle"
```

## 📊 Couverture des Tests

### Nœuds Testés

| Nœud | Services/Actions | Tests |
|------|------------------|-------|
| **curobo_fk** | `/curobo/fk_poses` | 7 tests |
| **curobo_ik** | 7 services | 15 tests |
| **curobo_gen_traj** | 5 services + 1 action | 12 tests |
| **curobo_gen_traj** (obstacles) | 6 services | 18 tests |
| **robot_segmentation** | 3 topics | 12 tests |
| **unified_planner** | 3 services + 1 action | 15 tests |
| **MPC planner** | Via unified_planner | 12 tests |
| **Planner switching** | Via unified_planner | 14 tests |
| **Pipeline complet** | Multi-nœuds | 9 tests |

**Total: ~114 tests**

### Services Couverts

#### curobo_fk (1 service)
- ✅ `/curobo/fk_poses` - Forward kinematics

#### curobo_ik (7 services)
- ✅ `/curobo_ik/ik_pose` - IK simple
- ✅ `/curobo_ik/ik_batch_poses` - IK batch
- ✅ `/curobo_ik/add_object` - Ajouter obstacle
- ✅ `/curobo_ik/remove_object` - Supprimer obstacle
- ✅ `/curobo_ik/remove_all_objects` - Supprimer tous
- ✅ `/curobo_ik/get_voxel_grid` - Grille voxel
- ✅ `/curobo_ik/get_collision_distance` - Distance collision

#### curobo_gen_traj (11 services)
- ✅ `/curobo_gen_traj/generate_trajectory` - Générer trajectoire
- ✅ `/curobo_gen_traj/is_available` - Disponibilité nœud
- ✅ `/curobo_gen_traj/set_robot_strategy` - Changer stratégie
- ✅ `/curobo_gen_traj/get_robot_strategy` - Obtenir stratégie
- ✅ `/curobo_gen_traj/update_motion_gen_config` - Mettre à jour config
- ✅ `/curobo_gen_traj/add_object` - Ajouter obstacle
- ✅ `/curobo_gen_traj/remove_object` - Supprimer obstacle
- ✅ `/curobo_gen_traj/remove_all_objects` - Supprimer tous
- ✅ `/curobo_gen_traj/get_obstacles` - Lister obstacles
- ✅ `/curobo_gen_traj/get_voxel_grid` - Grille voxel
- ✅ `/curobo_gen_traj/get_collision_distance` - Distance collision

#### unified_planner (3 services)
- ✅ `/unified_planner/generate_trajectory` - Générer trajectoire
- ✅ `/unified_planner/set_planner` - Changer planner
- ✅ `/unified_planner/list_planners` - Lister planners

### Actions Couvertes
- ✅ `/curobo_gen_traj/send_trajectory` - Exécution trajectoire
- ✅ `/unified_planner/execute_trajectory` - Exécution unifiée

### Topics Couverts
- ✅ `/masked_depth_image` - Image profondeur masquée
- ✅ `/collision_spheres` - Sphères de collision
- ✅ `/robot_pointcloud_debug` - Point cloud robot
- ✅ `/unified_planner/mpc_goal` - Mise à jour goal MPC

## 🧪 Types de Tests

### Tests Unitaires par Service
- Validation entrées/sorties
- Gestion erreurs
- Cas limites (vide, grand batch, invalide)
- Cohérence résultats

### Tests d'Intégration
- Pipeline FK → IK → Trajectoire
- Gestion obstacles + planification
- Changement dynamique de planner
- Exécution bout-en-bout

### Tests de Performance
- Temps de réponse services
- Latence MPC (< 100ms/step)
- Batch IK (1, 10, 100, 1000 poses)
- Changement planner (< 2s)

## 📝 Conventions de Test

### Nommage
```python
def test_<service_name>_<scenario>():
    """Description du test."""
    ...
```

### Structure
```python
def test_example():
    # Setup
    request = Service.Request()

    # Execute
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

    # Verify
    assert future.result() is not None
    assert future.result().success
```

### Fixtures
```python
from fixtures.test_poses import TestPoses, TestJointStates
from fixtures.test_robot_configs import TestRobotConfig

pose = TestPoses.reach_pose_1()
js = TestJointStates.home_state()
timeout = TestRobotConfig.SERVICE_TIMEOUT
```

## 🐛 Dépannage

### Nœuds non disponibles
```bash
# Vérifier les nœuds actifs
ros2 node list

# Vérifier les services
ros2 service list

# Vérifier les topics
ros2 topic list
```

### Tests timeout
```python
# Augmenter le timeout dans test_robot_configs.py
SERVICE_TIMEOUT = 20.0  # au lieu de 10.0
```

### Tests échouent
```bash
# Mode verbose pour debug
pytest tests/integration/test_fk_services.py -v -s

# Un seul test
pytest tests/integration/test_fk_services.py::TestFKServices::test_fk_single_joint_state -v
```

## 📚 Ressources

- [ROS2 Testing Guide](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Testing-Main.html)
- [pytest Documentation](https://docs.pytest.org/)
- [cuRobo Documentation](https://curobo.org/)

## ✅ Checklist Avant Commit

- [ ] Tous les tests passent: `pytest tests/integration/ -v`
- [ ] Pas de warnings: `pytest tests/integration/ -W error`
- [ ] Code formaté: `black tests/`
- [ ] Linting OK: `flake8 tests/`
- [ ] Documentation à jour
