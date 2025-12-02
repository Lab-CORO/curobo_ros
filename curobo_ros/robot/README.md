# Stratégies de contrôle robot

Ce dossier contient les différentes stratégies de contrôle pour différents types de robots.

## 📁 Structure

```
robot/
├── joint_control_strategy.py   # Interface abstraite (base)
├── robot_context.py             # Gestionnaire de stratégies + service de changement
├── doosan_strategy.py           # Stratégie pour Doosan M1013
├── emulator_strategy.py         # Émulateur de robot (simulation)
├── ghost_strategy.py            # Visualisation de trajectoire
└── README.md                    # Ce fichier
```

---

## 🎯 Stratégies disponibles

### 1. **DoosanControl** (`doosan_strategy.py`)

Contrôle du robot Doosan M1013 réel.

**Topics:**
- Publisher: `/leeloo/execute_trajectory` (JointTrajectory)
- Subscriber: `/leeloo/trajectory_state` (Float32)
- Subscriber: `/dsr01/joint_states` (JointState)

**Particularités:**
- Réordonnancement des joints (bug Doosan)
- Intégration avec le package `leeloo`

**Usage:**
```bash
ros2 param set /unified_planner robot_type "doosan_m1013"
```

---

### 2. **EmulatorStrategy** (`emulator_strategy.py`)

Émulateur de robot pour tests et visualisation sans matériel.

**Topics:**
- Publisher: `/joint_states` (JointState)

**Fonctionnement:**
- Simule l'exécution progressive d'une trajectoire
- Thread dédié pour progression temporelle
- Compatible avec RViz (robot principal)

**Usage:**
```bash
ros2 param set /unified_planner robot_type "emulator"
```

**Avantages:**
- ✅ Pas de robot physique requis
- ✅ Tests sécurisés
- ✅ CI/CD compatible
- ✅ Idéal pour développement

---

### 3. **GhostStrategy** (`ghost_strategy.py`)

Mode visualisation de trajectoire (preview).

**Topics:**
- Publisher: `/trajectory` (JointTrajectory)

**Fonctionnement:**
- Affiche instantanément la trajectoire planifiée
- Robot fantôme (avec préfixe `preview/`)
- Toujours actif en parallèle de la stratégie principale

**Usage:**
- Automatiquement actif pour toutes les stratégies
- Utilisé pour prévisualiser les trajectoires dans RViz

---

## 🔧 Ajouter une nouvelle stratégie

### Étape 1: Créer la classe

Créez un nouveau fichier `<robot>_strategy.py` :

```python
from curobo_ros.robot.joint_control_strategy import JointCommandStrategy, RobotState

class MyRobotControl(JointCommandStrategy):
    def __init__(self, node, dt):
        super().__init__(node, dt)
        # Votre initialisation

    def send_trajectrory(self):
        # Implémenter l'envoi de trajectoire
        pass

    def get_joint_pose(self):
        # Retourner la position actuelle
        pass

    def get_joint_name(self):
        # Retourner les noms des joints
        pass

    def stop_robot(self):
        # Arrêter le robot
        pass

    def get_progression(self):
        # Retourner la progression (0.0 à 1.0)
        pass
```

### Étape 2: Enregistrer dans robot_context.py

Ajoutez un case dans `select_strategy()` :

```python
def select_strategy(self, node, time_dilation_factor):
    robot_type = node.get_parameter('robot_type').get_parameter_value().string_value
    match robot_type:
        case "my_robot":
            from curobo_ros.robot.my_robot_strategy import MyRobotControl
            robot_strategy = MyRobotControl(node, time_dilation_factor)
        # ... autres cases
```

### Étape 3: Tester

```bash
ros2 param set /unified_planner robot_type "my_robot"
ros2 service call /unified_planner/set_robot_strategy std_srvs/srv/Trigger
```

---

## 📊 Comparaison des stratégies

| Aspect | Doosan | Emulator | Ghost |
|--------|--------|----------|-------|
| **Robot physique** | ✅ Requis | ❌ Non | ❌ Non |
| **Topic principal** | `/leeloo/execute_trajectory` | `/joint_states` | `/trajectory` |
| **Exécution** | Asynchrone | Simulée (thread) | Instantanée |
| **RViz** | Via driver Doosan | Robot principal | Robot preview |
| **Progression** | Via feedback | Simulée | Immédiate (100%) |
| **Use case** | Production | Développement/Test | Visualisation |

---

## 🔒 Thread-safety

Toutes les stratégies sont utilisées de manière thread-safe via `RobotContext.strategy_lock`.

Le changement de stratégie :
1. Acquiert le lock
2. Arrête le robot actuel
3. Crée la nouvelle stratégie
4. Switch le pointeur
5. Relâche le lock

---

## 🚀 Services disponibles

### Changer de stratégie

```bash
# 1. Modifier le paramètre
ros2 param set /unified_planner robot_type "<strategy_name>"

# 2. Activer le changement
ros2 service call /unified_planner/set_robot_strategy std_srvs/srv/Trigger
```

### Obtenir la stratégie actuelle

```bash
ros2 service call /unified_planner/get_robot_strategy std_srvs/srv/Trigger
```

---

## 📚 Documentation

Pour plus de détails, consultez :
- [Guide du changement dynamique de stratégie](../../doc/tutorials/dynamic_strategy_switching.md)
- [Architecture du système](../../doc/concepts/architecture.md)

---

## 🎓 Pattern Strategy

Ce module implémente le **pattern Strategy** (GoF) :

```
         JointCommandStrategy (interface)
                    ↑
        ┌───────────┼───────────┬───────────┐
        │           │           │           │
  DoosanControl  EmulatorStrategy  GhostStrategy  ...
```

**Avantages:**
- ✅ Facilite l'ajout de nouveaux robots
- ✅ Changement à chaud sans recompilation
- ✅ Code découplé et testable
- ✅ Stratégies interchangeables
