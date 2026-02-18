# Liste des dépendances au package Doosan

**Date**: 2025-11-05
**Branche analysée**: claude/doosan-dependencies-list-011CUq4aLC4wYwKGRkaz3w6z
**Note**: La branche `disconnect_doosan` n'existe pas dans le dépôt.

---

## 📋 Vue d'ensemble

Le projet curobo_ros a **16 fichiers** contenant des références au package doosan. Ces dépendances sont critiques pour le fonctionnement avec le robot Doosan M1013.

---

## 1. Submodule Git

**Fichier**: `.gitmodules`
**Lignes**: 1-3

```
[submodule "curobo_doosan"]
    path = curobo_doosan
    url = https://github.com/Lab-CORO/curobo_doosan.git
```

**Criticité**: 🔴 **Haute**
**Note**: Le submodule existe mais n'est pas actuellement initialisé (répertoire vide).

---

## 2. Installation et Configuration

### `setup.py`
**Lignes**: 16-18

```python
(os.path.join('share', package_name, 'curobo_doosan/src/m1013'),
    glob(os.path.join('curobo_doosan/src/m1013', '*.*'))),
(os.path.join('share', package_name, 'curobo_doosan/src/m1013/meshes', 'm1013_white'),
    glob(os.path.join('curobo_doosan/src/m1013/meshes/m1013_white', '*.dae*'))),
(os.path.join('share', package_name, 'curobo_doosan/src/m1013/meshes', 'm1013_collision'),
    glob(os.path.join('curobo_doosan/src/m1013/meshes/m1013_collision', '*.dae*'))),
```

**Criticité**: 🔴 **Haute**
**Fichiers requis**:
- Tous les fichiers dans `curobo_doosan/src/m1013/`
- Meshes de visualisation: `curobo_doosan/src/m1013/meshes/m1013_white/*.dae`
- Meshes de collision: `curobo_doosan/src/m1013/meshes/m1013_collision/*.dae`

---

## 3. Stratégie de contrôle du robot Doosan

### `curobo_ros/robot/doosan_strategy.py`
**Lignes**: 1-109 (fichier complet)

**Classe principale**: `DoosanControl` (hérite de `JointCommandStrategy`)

**Topics ROS spécifiques**:
- **Publisher**: `/leeloo/execute_trajectory` (JointTrajectory)
- **Subscriber**: `/leeloo/trajectory_state` (Float32)
- **Subscriber**: `/dsr01/joint_states` (JointState) ⚠️ **Topic spécifique Doosan**

**Criticité**: 🔴 **Haute**

**Particularité importante**: Réordonnancement des joints (lignes 92-104)
```python
# Ligne 97: Stupidities from doosan cf joint_states msg
self.joint_pose = [
    joint_pose_msg.position[0],
    joint_pose_msg.position[1],
    joint_pose_msg.position[4],  # ⚠️ Position 4 devient 2
    joint_pose_msg.position[2],  # ⚠️ Position 2 devient 3
    joint_pose_msg.position[3],  # ⚠️ Position 3 devient 4
    joint_pose_msg.position[5]
]
```

**Impact**: Le mapping des joints Doosan est non-standard et nécessite une réorganisation.

---

## 4. Contexte Robot

### `curobo_ros/robot/robot_context.py`
**Lignes**: 13, 24-27

```python
# Ligne 13: Paramètre par défaut
node.declare_parameter('robot_type', "doosan_m1013")

# Lignes 24-27: Import conditionnel
match robot_type:
    case "doosan_m1013":
        from curobo_ros.robot.doosan_strategy import DoosanControl
        robot_strategy = DoosanControl(node, time_dilation_factor)
```

**Criticité**: 🟡 **Moyenne**
**Note**: Import conditionnel, donc peut être modifié pour supporter d'autres robots.

---

## 5. Wrapper de Configuration

### `curobo_ros/core/config_wrapper.py`
**Lignes**: 61, 70

```python
# Ligne 61: Fichier de configuration par défaut
node.declare_parameter('robot_config_file',
    os.path.join(package_share_directory, 'curobo_doosan', 'src', 'm1013', 'm1013.yml'))

# Ligne 70: Chemin URDF
urdf_file = os.path.join(package_share_directory, 'curobo_doosan', 'src', 'm1013', urdf_file)
```

**Criticité**: 🔴 **Haute**
**Fichiers requis**:
- `curobo_doosan/src/m1013/m1013.yml`
- `curobo_doosan/src/m1013/<urdf_file>` (nom défini dans le yml)

---

## 6. Segmentation Robot

### `curobo_ros/core/robot_segmentation.py`
**Lignes**: 39, 48, 66-67

```python
# Ligne 39: Configuration
self.declare_parameter('robot_config_file',
    os.path.join(package_share_directory, 'curobo_doosan', 'src', 'm1013', 'm1013.yml'))

# Ligne 48: URDF
urdf_file = os.path.join(package_share_directory, 'curobo_doosan', 'src', 'm1013', urdf_file)

# Ligne 66-67: Commentaire révélateur
# Robot strategie because of doosan stupidities...
self.robot_context = RobotContext(self, 0.03)
```

**Criticité**: 🔴 **Haute**
**Dépendance indirecte**: Utilise `RobotContext` qui charge `DoosanControl`

---

## 7. Forward Kinematics

### `curobo_ros/core/fk.py`
**Ligne**: 36

```python
config_file_path = os.path.join(
    get_package_share_directory("curobo_ros"),
    'curobo_doosan/src/m1013/m1013.yml'
)
```

**Criticité**: 🔴 **Haute**
**Fichier requis**: `curobo_doosan/src/m1013/m1013.yml`

---

## 8. Launch Files

### `launch/launch_rviz2.launch.py`
**Ligne**: 11

```python
urdf = Command(['cat ', PathJoinSubstitution([
    FindPackageShare('curobo_ros'),
    'curobo_doosan/src/m1013/',
    urdf_file_name
])])
```

**Criticité**: 🔴 **Haute**
**Fichier requis**: `curobo_doosan/src/m1013/m1013.urdf`

---

### `launch/gen_traj.launch.py`
**Ligne**: 22

```python
urdf = Command(['cat ', PathJoinSubstitution([
    FindPackageShare('curobo_ros'),
    'curobo_doosan/src/m1013/',
    urdf_file_name
])])
```

**Criticité**: 🔴 **Haute**
**Fichier requis**: `curobo_doosan/src/m1013/m1013.urdf`

---

## 9. Documentation

### Fichiers de documentation
- `README.md:20` - Lien vers tutoriel doosan
- `doc/getting-started/installation.md:70` - Lien vers tutoriel doosan
- `doc/tutorials/doosan_example.md` - **Tutoriel complet** sur le Doosan M1013
- `doc/concepts/ros_interfaces.md:33` - Documentation du paramètre `robot_config_file`
- `doc/concepts/architecture.md:24,33,41` - Diagrammes incluant `DoosanControl`

**Criticité**: 🟢 **Basse**
**Note**: Documentation uniquement, pas d'impact sur le code.

---

## 10. Ghost Strategy

### `curobo_ros/robot/ghost_strategy.py`
**Ligne**: 7

```python
'''
This class is a strategie to control doosan robot with motion_gen
'''
```

**Criticité**: 🟢 **Basse**
**Note**: Seulement un commentaire (probablement copié-collé de `doosan_strategy.py`)

---

## 📊 Résumé par type de dépendance

| Type de dépendance | Nombre | Criticité | Impact |
|-------------------|--------|-----------|---------|
| **Fichiers de configuration YAML** | 4 occurrences | 🔴 Haute | Bloque le démarrage |
| **Fichiers URDF** | 3 occurrences | 🔴 Haute | Bloque la visualisation |
| **Meshes 3D (DAE)** | 2 types | 🔴 Haute | Bloque les collisions |
| **Classe DoosanControl** | 1 classe | 🔴 Haute | Bloque le contrôle robot |
| **Topics ROS** | 1 topic | 🟡 Moyenne | `/dsr01/joint_states` |
| **Paramètres ROS** | 2 paramètres | 🟡 Moyenne | `robot_type`, `robot_config_file` |
| **Documentation** | 5 fichiers | 🟢 Basse | Aucun impact fonctionnel |

---

## 🎯 Résumé des fichiers critiques du submodule

Fichiers requis dans `curobo_doosan/src/m1013/`:
1. ✅ `m1013.yml` - Configuration robot (4 références)
2. ✅ `m1013.urdf` - Description robot (3 références)
3. ✅ `meshes/m1013_white/*.dae` - Meshes de visualisation
4. ✅ `meshes/m1013_collision/*.dae` - Meshes de collision

---

## ⚠️ Points d'attention pour la déconnexion

### 1. Réorganisation des joints
Le code contient un mapping spécifique pour les "stupidités" du format Doosan :
- `doosan_strategy.py:97` - Réordonnancement : `[0,1,4,2,3,5]` au lieu de `[0,1,2,3,4,5]`
- Ce comportement est **critique** pour le contrôle du robot

### 2. Topics ROS spécifiques
- `/dsr01/joint_states` - Topic avec namespace Doosan
- `/leeloo/execute_trajectory` - Interface avec le package leeloo
- `/leeloo/trajectory_state` - Feedback d'exécution

### 3. Dépendances externes
- Package **leeloo** (https://github.com/Lab-CORO/leeloo) pour l'exécution de trajectoires
- Mentionné dans `doc/tutorials/doosan_example.md:26`

---

## 🔧 Recommandations pour la déconnexion

Pour découpler le code du package doosan :

1. **Abstraction de la configuration**
   - Créer une structure de configuration générique
   - Remplacer les chemins hardcodés par des paramètres

2. **Stratégie de contrôle**
   - Garder `DoosanControl` comme une stratégie optionnelle
   - S'assurer que d'autres stratégies (UR5e, etc.) peuvent être utilisées

3. **Fichiers de ressources**
   - Déplacer les fichiers URDF/YAML vers un système de plugins
   - Ou utiliser un package séparé pour chaque type de robot

4. **Documentation**
   - Mettre à jour les tutoriels pour être robot-agnostiques
   - Créer des exemples pour d'autres robots

---

## 📝 Notes additionnelles

- Le submodule `curobo_doosan` pointe vers : https://github.com/Lab-CORO/curobo_doosan.git
- Le répertoire est actuellement vide (non initialisé)
- La stratégie UR5e est mentionnée mais pas encore implémentée (`robot_context.py:28-29`)

---

**Fin de l'analyse**
