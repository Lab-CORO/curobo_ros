# Rosbags de Test pour CuRobo ROS

Ce dossier contient les rosbags nécessaires pour les tests d'intégration du projet curobo_ros.

## 📁 Structure

```
rosbags/
├── perception/              # Tests de perception caméra
│   ├── test_depth_static_scene.*
│   ├── test_depth_dynamic_obstacle.*
│   ├── test_depth_robot_motion.*
│   └── test_pointcloud_workspace.*
├── robot_control/           # Tests de contrôle robot
│   ├── test_robot_trajectory_execution.*
│   └── test_robot_joint_states_variety.*
├── visualization/           # Tests de visualisation
│   └── test_visualization_markers.*
├── e2e_scenarios/           # Scénarios end-to-end
│   ├── test_e2e_pick_place.*
│   └── test_e2e_obstacle_avoidance.*
└── stress/                  # Tests de performance
    └── test_high_frequency_data.*
```

## 📊 Rosbags Requis

### Priorité P0 (Essentiel - CI/CD)

1. **test_depth_static_scene.bag** (~500 MB)
   - Scène statique avec objets pour validation segmentation
   - Topics: depth, camera_info, joint_states, tf
   - Durée: 30s

2. **test_robot_joint_states_variety.bag** (~100 MB)
   - Diverses configurations robot pour tests FK/IK
   - Topics: joint_states, tf
   - Durée: 60s

3. **test_robot_trajectory_execution.bag** (~50 MB)
   - Exécution complète de trajectoire
   - Topics: joint_states, execute_trajectory, trajectory_state
   - Durée: 30s

### Priorité P1 (Tests Avancés)

4. test_depth_dynamic_obstacle.bag (~1 GB)
5. test_e2e_pick_place.bag (~3 GB)
6. test_pointcloud_workspace.bag (~2 GB)

### Priorité P2 (Tests Spécifiques)

7. test_visualization_markers.bag (~100 MB)
8. test_high_frequency_data.bag (~10 GB)
9. test_e2e_obstacle_avoidance.bag (~2.5 GB)

**Total estimé**: ~20-25 GB

## 🎥 Enregistrement des Rosbags

### Commandes d'Enregistrement

Voir `INTEGRATION_TEST_PLAN.md` section "Commandes pour Enregistrer les Rosbags" pour les commandes complètes.

Exemple pour scène statique:
```bash
ros2 bag record -o test_depth_static_scene \
  /depth_to_rgb/image_raw \
  /depth_to_rgb/camera_info \
  /dsr01/joint_states \
  /tf /tf_static \
  --duration 30
```

### Checklist avant Enregistrement

- [ ] Calibration caméra vérifiée
- [ ] TF tree complet
- [ ] Éclairage stable
- [ ] Espace de travail dégagé
- [ ] Robot homed
- [ ] Stockage >10GB disponible

## 📝 Format des Métadonnées

Chaque rosbag `.db3` doit avoir un fichier `.yaml` associé contenant:

```yaml
rosbag:
  name: test_depth_static_scene
  version: 1.0
  date_recorded: 2025-11-10
  duration: 30.0

hardware:
  robot: Doosan M1013
  camera: RealSense D435

environment:
  lighting: normal_indoor
  objects:
    - name: "box_1"
      type: "cube"
      dimensions: [0.1, 0.1, 0.1]
      position: [0.5, 0.2, 0.0]

test_compatibility:
  - TEST_DEPTH_001
  - TEST_DEPTH_002
```

## ✅ Validation

Valider un rosbag avant utilisation:

```bash
python3 scripts/validate_rosbag.py \
  test/rosbags/perception/test_depth_static_scene.db3 \
  test/rosbags/perception/test_depth_static_scene.yaml
```

Vérifications:
- Tous les topics requis présents
- Durée correcte
- Timestamps synchronisés
- Pas de frame drops

## 💾 Stockage

### Option 1: Git LFS

```bash
git lfs install
git lfs track "test/rosbags/**/*.db3"
git add test/rosbags/
git commit -m "Add test rosbags"
```

### Option 2: Stockage Cloud

Si les fichiers sont hébergés ailleurs (Google Drive, S3, NAS):

```bash
./scripts/download_test_data.sh
```

## 🧪 Utilisation dans les Tests

```python
import pytest
from rosbag2_py import Player

@pytest.fixture
def rosbag_player():
    player = Player()
    player.play('test/rosbags/perception/test_depth_static_scene.db3')
    yield player
    player.stop()

def test_with_rosbag(rosbag_player):
    # Test utilisant les données du rosbag
    pass
```

## 📖 Documentation Complète

Voir `INTEGRATION_TEST_PLAN.md` pour:
- Spécifications détaillées de chaque rosbag
- Scénarios d'enregistrement
- Scripts de validation
- Intégration dans CI/CD

## 🔗 Liens Utiles

- [Plan de Test d'Intégration](../../INTEGRATION_TEST_PLAN.md)
- [Documentation ROS 2 rosbag2](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)

---

**Note**: Les rosbags ne sont pas commités dans ce repository par défaut.
Téléchargez-les depuis [LIEN_VERS_STOCKAGE] ou enregistrez-les localement.
