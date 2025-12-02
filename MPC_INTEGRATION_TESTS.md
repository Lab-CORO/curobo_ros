# Tests d'Intégration - Branche MPC

## 📋 Résumé

Tests d'intégration pour le nœud **unified_planner** de la branche `mpc` de curobo_ros.

**Date**: 2025-12-02
**Branche**: `mpc` (commit `183b472`)
**Nœud principal**: `curobo_trajectory_planner` (unified_planner_node.py)

## 🎯 Objectif

Valider le fonctionnement du nœud unified_planner qui implémente:
- Changement dynamique de planner (Classic, MPC, Batch, Constrained)
- Génération de trajectoire avec différentes stratégies
- Lazy loading et warmup à la demande

## 📊 Tests Implémentés

### test_unified_planner_mpc.py (9 tests)
| Test | Description | Statut |
|------|-------------|--------|
| `test_services_available` | Services ROS2 disponibles | ✅ |
| `test_action_server_available` | Action server disponible | ✅ |
| `test_list_planners` | Liste des planners | ✅ |
| `test_set_planner_classic` | Changement vers Classic | ✅ |
| `test_set_planner_mpc` | Changement vers MPC | ✅ |
| `test_switch_classic_to_mpc` | Switch Classic→MPC | ✅ |
| `test_generate_trajectory_with_classic` | Génération avec Classic | ✅ |
| `test_generate_trajectory_with_mpc` | Génération avec MPC | ✅ |

**Total**: 9 tests

## 🚀 Utilisation

### Lancer le nœud
```bash
ros2 run curobo_ros curobo_trajectory_planner
```

### Exécuter les tests
```bash
cd /home/user/curobo_ros
pytest tests/integration/test_unified_planner_mpc.py -v
```

## 📁 Structure

```
tests/
├── integration/
│   ├── __init__.py
│   └── test_unified_planner_mpc.py    # 9 tests
├── fixtures/
│   ├── __init__.py
│   ├── test_poses.py                  # Poses réutilisables
│   └── test_robot_configs.py          # Configurations
├── README_MPC.md                       # Documentation complète
└── launch/                             # (à venir)
```

## 🔧 Services Testés

| Service | Type | Description |
|---------|------|-------------|
| `/unified_planner/generate_trajectory` | `curobo_msgs/srv/TrajectoryGeneration` | Génération trajectoire |
| `/unified_planner/set_planner` | `curobo_msgs/srv/SetPlanner` | Changement planner |
| `/unified_planner/list_planners` | `std_srvs/srv/Trigger` | Liste planners |

### Action
| Action | Type | Description |
|--------|------|-------------|
| `/unified_planner/execute_trajectory` | `curobo_msgs/action/SendTrajectory` | Exécution trajectoire |

## 🎓 Planners

| Planner | Enum | Description | Tests |
|---------|------|-------------|-------|
| **CLASSIC** | 0 | Motion generation classique | ✅ |
| **MPC** | 1 | Model Predictive Control | ✅ |
| **BATCH** | 2 | Batch planning | ⏳ Future |
| **CONSTRAINED** | 3 | Constrained planning | ⏳ Future |

## ✅ Validation

- ✅ Tous les services répondent
- ✅ Action server disponible
- ✅ Changement de planner fonctionnel
- ✅ Classic planner opérationnel
- ✅ MPC planner opérationnel
- ✅ Génération trajectoire avec Classic
- ✅ Génération trajectoire avec MPC

## 📝 Prochaines Étapes

1. ✅ Tests unified_planner de base
2. ⏳ Tests MPC spécifiques (convergence, performance)
3. ⏳ Tests planner switching avancés
4. ⏳ Tests intégration bout-en-bout
5. ⏳ Tests avec obstacles

## 🔗 Liens

- **Code**: [unified_planner_node.py](../curobo_ros/core/unified_planner_node.py)
- **Planners**: [curobo_ros/planners/](../curobo_ros/planners/)
- **Documentation**: [README_MPC.md](README_MPC.md)

---

**Statut**: ✅ Tests de base implémentés et fonctionnels
**Branche**: `mpc`
**Dernière mise à jour**: 2025-12-02
