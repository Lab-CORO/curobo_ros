# Services personnalisés - Installation

## 📋 Service SetRobotStrategy

Le service `SetRobotStrategy.srv` permet de changer dynamiquement la stratégie de contrôle du robot.

### Fichier de service

**Emplacement:** `srv/SetRobotStrategy.srv`

```
# Service to change the robot control strategy dynamically
# Available strategies: "doosan_m1013", "ur5e", "emulator", "ghost"

# Request
string strategy_name    # Name of the strategy to switch to

---

# Response
bool success           # True if strategy was changed successfully
string message         # Status message or error description
string previous_strategy  # Name of the previous strategy
```

---

## 🔧 Installation dans le package

### Option 1: Si le package utilise ament_cmake

Si `curobo_ros` utilise CMake, ajoutez dans `CMakeLists.txt` :

```cmake
# Find dependencies
find_package(rosidl_default_generators REQUIRED)

# Declare ROS 2 messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetRobotStrategy.srv"
  # ... autres services
  DEPENDENCIES std_msgs
)
```

### Option 2: Si le package utilise ament_python

Si `curobo_ros` utilise ament_python (comme actuellement), le service devrait être défini dans un **package séparé** comme `curobo_msgs`.

#### Créer le service dans curobo_msgs

1. **Placer le fichier dans curobo_msgs:**
   ```bash
   # Dans le package curobo_msgs
   mkdir -p srv
   cp SetRobotStrategy.srv curobo_msgs/srv/
   ```

2. **Modifier CMakeLists.txt de curobo_msgs:**
   ```cmake
   rosidl_generate_interfaces(${PROJECT_NAME}
     # ... services existants ...
     "srv/SetRobotStrategy.srv"
     DEPENDENCIES std_msgs
   )
   ```

3. **Recompiler curobo_msgs:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select curobo_msgs
   source install/setup.bash
   ```

---

## 📝 Note: Workaround actuel

Comme `SetRobotStrategy.srv` n'est pas encore compilé dans `curobo_msgs`, l'implémentation actuelle utilise **`std_srvs/srv/Trigger`** comme interface temporaire.

### Utilisation avec Trigger

Le service fonctionne actuellement ainsi :

1. **Modifier le paramètre** `robot_type` avec la stratégie souhaitée
2. **Appeler le service** Trigger pour activer le changement

```bash
# 1. Set parameter
ros2 param set /curobo_gen_traj robot_type "ur5e"

# 2. Trigger strategy switch
ros2 service call /curobo_gen_traj/set_robot_strategy std_srvs/srv/Trigger
```

**Avantages du workaround:**
- ✅ Fonctionne immédiatement sans recompilation
- ✅ Compatible avec les messages standards ROS
- ✅ Facile à tester

**Inconvénients:**
- ⚠️ Nécessite deux étapes (paramètre + service)
- ⚠️ Pas de validation du nom de stratégie dans la requête

---

## 🚀 Migration vers SetRobotStrategy (futur)

Une fois le service compilé dans `curobo_msgs`, modifier `robot_context.py` :

```python
# Remplacer:
from std_srvs.srv import Trigger

# Par:
from curobo_msgs.srv import SetRobotStrategy

# Et modifier le service:
self.set_strategy_srv = node.create_service(
    SetRobotStrategy,  # Au lieu de Trigger
    node.get_name() + '/set_robot_strategy',
    partial(self.set_robot_strategy_callback, node)
)

# Adapter le callback:
def set_robot_strategy_callback(self, node, request, response):
    new_strategy_name = request.strategy_name  # Lire depuis request
    # ... reste du code ...
    response.previous_strategy = previous_strategy
    return response
```

### Avantages de SetRobotStrategy

- ✅ Interface plus claire (nom de stratégie dans la requête)
- ✅ Validation possible côté service
- ✅ Retourne la stratégie précédente
- ✅ Une seule étape pour l'utilisateur

---

## 🧪 Test du service

### Test avec Trigger (actuel)

```bash
# Terminal 1: Lancer le node
ros2 launch curobo_ros gen_traj.launch.py

# Terminal 2: Changer de stratégie
ros2 param set /curobo_gen_traj robot_type "ghost"
ros2 service call /curobo_gen_traj/set_robot_strategy std_srvs/srv/Trigger

# Vérifier
ros2 service call /curobo_gen_traj/get_robot_strategy std_srvs/srv/Trigger
```

### Test avec SetRobotStrategy (futur)

```bash
# Une seule commande suffit
ros2 service call /curobo_gen_traj/set_robot_strategy curobo_msgs/srv/SetRobotStrategy "{strategy_name: 'ghost'}"
```

---

## 📚 Références

- Documentation complète: [dynamic_strategy_switching.md](../doc/tutorials/dynamic_strategy_switching.md)
- Implémentation: `curobo_ros/robot/robot_context.py`
