# Guide de test des planners

## Architecture

Chaque planner extrait les données dont il a besoin de la request `TrajectoryGeneration` :

| Planner | Champ utilisé | Description |
|---------|---------------|-------------|
| **ClassicPlanner** | `target_pose` | Planification vers une seule pose |
| **MPCPlanner** | `target_pose` | Contrôle MPC vers une seule pose |
| **MultiPointPlanner** | `target_poses` | Planification à travers plusieurs waypoints |

## Message TrajectoryGeneration

Le service accepte deux champs :
```
geometry_msgs/Pose target_pose          # Pour ClassicPlanner et MPCPlanner
geometry_msgs/Pose[] target_poses       # Pour MultiPointPlanner
```

---

## 1. ClassicPlanner - Planification simple

### Commande ROS
```bash
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration "
target_pose:
  position: {x: 0.5, y: 0.0, z: 0.4}
  orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}
"
```

### Exemple avec orientation différente
```bash
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration "
target_pose:
  position: {x: 0.45, y: 0.1, z: 0.5}
  orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}
"
```

---

## 2. MultiPointPlanner - Planification multi-waypoints

**Note:** Ce planner utilise l'approche de `pose_sequence_example.py` de cuRobo :
- Planifie séquentiellement chaque segment
- Empile (stack) les trajectoires pour créer une trajectoire continue
- Remet les vitesses/accélérations à zéro entre waypoints
- Retourne une trajectoire complète combinée

### Exemple : 2 waypoints (approche + cible)
```bash
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration "
target_poses:
- position: {x: 0.45, y: 0.1, z: 0.6}
  orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}
- position: {x: 0.5, y: 0.1, z: 0.4}
  orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}
"
```

### Exemple : 3 waypoints (approche + préhension + retrait)
```bash
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration "
target_poses:
- {position: {x: 0.4, y: 0.2, z: 0.5}, orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}}
- {position: {x: 0.5, y: 0.0, z: 0.3}, orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}}
- {position: {x: 0.4, y: -0.2, z: 0.5}, orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}}
"
```

### Exemple : 4 waypoints (pick and place)
```bash
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration "
target_poses:
- {position: {x: 0.35, y: 0.15, z: 0.6}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}
- {position: {x: 0.4, y: 0.15, z: 0.35}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}
- {position: {x: 0.35, y: 0.15, z: 0.6}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}
- {position: {x: 0.5, y: -0.15, z: 0.45}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}
"
```

---

## 3. MPCPlanner - Contrôle en boucle fermée

### Commande ROS
```bash
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration "
target_pose:
  position: {x: 0.5, y: 0.0, z: 0.4}
  orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}
"
```

**Note:** MPCPlanner utilise `target_pose` comme ClassicPlanner, mais l'exécution est différente (closed-loop vs open-loop).

---

## Workflow complet de test

### 1. Sélectionner le planner
```bash
# Pour ClassicPlanner
ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 0}"

# Pour MPCPlanner
ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 1}"

# Pour MultiPointPlanner (après l'avoir ajouté au registry)
ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 2}"
```

### 2. Générer la trajectoire
```bash
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration "..."
```

### 3. Exécuter la trajectoire
```bash
ros2 action send_goal /unified_planner/execute_trajectory curobo_msgs/action/SendTrajectory "{}"
```

---

## Notes importantes

### Orientations (quaternions)

Quelques orientations courantes :

| Orientation | Quaternion (x, y, z, w) | Description |
|-------------|-------------------------|-------------|
| Verticale (vers le bas) | `0, 1, 0, 0` | End-effector pointant vers le bas |
| Inclinée 90° | `0, 0.707, 0, 0.707` | Inclinée de 90° |
| Horizontale | `0, 0, 0, 1` | End-effector horizontal |

### Positions

- **x** : Distance avant/arrière du robot (positif = avant)
- **y** : Distance gauche/droite (positif = gauche)
- **z** : Hauteur (positif = haut)

### Valeurs typiques pour le Doosan M1013
- x: 0.3 à 0.8 m
- y: -0.5 à 0.5 m
- z: 0.2 à 0.8 m

---

## Enregistrer MultiPointPlanner dans le PlannerFactory

Pour utiliser MultiPointPlanner, il faut l'enregistrer dans `planner_factory.py` :

```python
from .multi_point_planner import MultiPointPlanner

_PLANNER_REGISTRY = {
    'classic': ClassicPlanner,
    'motion_gen': ClassicPlanner,
    'mpc': MPCPlanner,
    'multi_point': MultiPointPlanner,  # Ajouter cette ligne
}
```

Ensuite dans `unified_planner_node.py`, ajouter le type d'enum correspondant.

---

## Debugging

### Vérifier le planner actuel
```bash
ros2 service call /unified_planner/list_planners std_srvs/srv/Trigger
```

### Logs
Les logs montrent quel planner est utilisé :
```
[INFO] Planning with Classic Motion Generation
[INFO] Planning through 3 waypoints with max_attempts=2, timeout=10.0s
```

---

## Résumé

✅ **ClassicPlanner** → `target_pose` → Planification simple
✅ **MPCPlanner** → `target_pose` → Contrôle adaptatif
✅ **MultiPointPlanner** → `target_poses` → Multiple waypoints

Chaque planner extrait automatiquement les données dont il a besoin de la request !
