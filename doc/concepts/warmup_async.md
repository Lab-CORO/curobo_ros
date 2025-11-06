# Warmup asynchrone et tracking de progression

## 📋 Vue d'ensemble

Le warmup de cuRobo est un processus intensif qui peut prendre **30 à 60 secondes**. Pour éviter de bloquer toute l'application pendant ce temps, le système a été modifié pour exécuter le warmup dans un **thread séparé** avec **tracking de progression en temps réel**.

## 🎯 Fonctionnalités

### 1. **Warmup asynchrone (non-bloquant)**
- Le warmup s'exécute dans un thread en arrière-plan
- Le node ROS démarre **immédiatement**
- RViz et les autres nodes restent **responsifs**
- Les services sont disponibles mais **attendent automatiquement** la fin du warmup

### 2. **Tracking de progression**
- **Estimation temporelle** basée sur la configuration
- **Publisher ROS** pour suivi en temps réel
- **Paramètre ROS** pour interrogation de l'état
- **Logs périodiques** tous les 5 secondes

### 3. **Protection des services**
- Les services **bloquent automatiquement** si warmup non terminé
- Attente de **maximum 60 secondes**
- Message d'erreur clair si timeout
- Service `is_available` pour vérifier l'état

---

## 🚀 Utilisation

### Monitoring de la progression

#### Option 1 : Via topic ROS (Recommandé)
```bash
# Terminal 1: Lancer le node
ros2 launch curobo_ros gen_traj.launch.py

# Terminal 2: Monitorer la progression en temps réel
ros2 topic echo /curobo_gen_traj/warmup_progress
```

**Output exemple:**
```
data: 0.15    # 15%
---
data: 0.34    # 34%
---
data: 0.58    # 58%
---
data: 0.87    # 87%
---
data: 1.0     # 100% - Terminé!
```

#### Option 2 : Via paramètre ROS
```bash
# Interroger le paramètre de progression
ros2 param get /curobo_gen_traj warmup_progress

# Interroger la disponibilité du node
ros2 param get /curobo_gen_traj node_is_available
```

#### Option 3 : Via service
```bash
# Vérifier si le node est disponible
ros2 service call /curobo_gen_traj/is_available std_srvs/srv/Trigger
```

**Response:**
```yaml
success: true   # ou false si warmup non terminé
message: ''
```

### Exemple Python pour attendre le warmup

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
import time

class WarmupMonitor(Node):
    def __init__(self):
        super().__init__('warmup_monitor')
        self.warmup_complete = False

        # S'abonner au topic de progression
        self.sub = self.create_subscription(
            Float32,
            '/curobo_gen_traj/warmup_progress',
            self.progress_callback,
            10
        )

    def progress_callback(self, msg):
        progress = msg.data * 100
        self.get_logger().info(f'Warmup: {progress:.1f}%')

        if msg.data >= 1.0:
            self.warmup_complete = True
            self.get_logger().info('✅ Warmup complete!')

    def wait_for_warmup(self, timeout=60.0):
        '''Attend la fin du warmup'''
        start_time = time.time()

        while not self.warmup_complete:
            if time.time() - start_time > timeout:
                self.get_logger().error('❌ Warmup timeout!')
                return False

            rclpy.spin_once(self, timeout_sec=0.1)

        return True

def main():
    rclpy.init()
    monitor = WarmupMonitor()

    print('Waiting for warmup...')
    if monitor.wait_for_warmup(timeout=120.0):
        print('Ready to use curobo_gen_traj!')
    else:
        print('Warmup failed or timeout')

    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 📊 Estimation de la durée du warmup

La durée du warmup est **estimée automatiquement** en fonction de la configuration :

| Paramètre | Impact sur la durée |
|-----------|---------------------|
| `use_cuda_graph = True` | **+25 secondes** (compilation CUDA) |
| `num_trajopt_seeds = 12` | **+18 secondes** (1.5s par seed) |
| `num_graph_seeds = 12` | **+9.6 secondes** (0.8s par seed) |
| `trajopt_tsteps = 32` | **+1 seconde** (0.3s par 10 steps) |
| `collision_checker_type = BLOX` | **+5 secondes** |
| **Base** | **10 secondes** |

**Exemple de calcul:**
```python
use_cuda_graph = True          # +25s
num_trajopt_seeds = 12         # +18s
num_graph_seeds = 12           # +9.6s
trajopt_tsteps = 32            # +1s
collision_checker_type = BLOX  # +5s
base_time = 10s

Total estimé = 10 + 25 + 18 + 9.6 + 1 + 5 = 68.6 secondes
```

**Courbe de progression (S-curve):**
- **0-40% du temps** → 0-20% de progression (Compilation CUDA)
- **40-90% du temps** → 20-80% de progression (Warmup tensors)
- **90-100% du temps** → 80-100% de progression (Finalisation)

---

## 🔧 Architecture technique

### Threads créés

1. **Thread principal** : Node ROS (non bloqué)
2. **Thread warmup** : Exécute `motion_gen.warmup()`
3. **Thread monitoring** : Met à jour la progression toutes les 0.5s

### Synchronisation

- **`threading.Event`** : Signal de fin de warmup
- **`threading.Lock`** : Protection des accès concurrents
- **Paramètres ROS** : État partagé thread-safe

### Protection des services

Tous les services critiques utilisent le décorateur `@require_warmup_complete` :

```python
@require_warmup_complete
def callback_add_object(self, node, request, response):
    # Ce service attendra automatiquement la fin du warmup
    # si appelé trop tôt
    ...
```

**Services protégés:**
- `/add_object`
- `/remove_object`
- `/remove_all_objects`
- `/get_voxel_grid`
- `/get_collision_distance`
- `/generate_trajectory`

**Services toujours disponibles:**
- `/is_available` (pour vérifier l'état)
- `/get_obstacles` (lecture uniquement)

---

## ⚠️ Comportement en cas d'appel prématuré

Si un service est appelé **avant la fin du warmup** :

1. Le service **affiche un warning** dans les logs
2. Le service **attend automatiquement** (max 60 secondes)
3. Si timeout : retourne `success=False` avec message d'erreur
4. Si warmup OK : exécute normalement

**Exemple de logs:**
```
[WARN] [curobo_gen_traj]: Service called but warmup not complete. Waiting for warmup to finish...
[INFO] [curobo_gen_traj]: Warmup progress: 45.2% (30.5s / 68.6s)
[INFO] [curobo_gen_traj]: Warmup progress: 67.8% (35.5s / 68.6s)
[INFO] [curobo_gen_traj]: ✅ Warmup complete in 42.3s (estimated: 68.6s)
[INFO] [curobo_gen_traj]: Service executed successfully
```

---

## 🐛 Debugging

### Warmup trop long

Si le warmup prend plus de temps que prévu :

```bash
# Vérifier les logs pour voir l'estimation
ros2 launch curobo_ros gen_traj.launch.py

# Dans les logs, chercher:
# "Warmup thread: Starting warmup (estimated duration: XX.Xs)"
```

**Solutions:**
- Réduire `num_trajopt_seeds` (par défaut: 12)
- Réduire `num_graph_seeds` (par défaut: 12)
- Réduire `trajopt_tsteps` (par défaut: 32)
- Mettre `use_cuda_graph = False` (gain de 25s, mais perte de performance)

### Warmup bloqué

Si la progression reste à 0% :

```bash
# Vérifier les erreurs CUDA
ros2 topic echo /curobo_gen_traj/warmup_progress

# Vérifier les logs d'erreur
# Si aucune progression après 10s, probablement une erreur CUDA
```

**Vérifier:**
- GPU disponible : `nvidia-smi`
- CUDA installé : `nvcc --version`
- PyTorch avec CUDA : `python3 -c "import torch; print(torch.cuda.is_available())"`

### Service timeout

Si un service retourne "warmup timeout" :

```bash
# Vérifier l'état actuel
ros2 param get /curobo_gen_traj node_is_available

# Si False, le warmup n'est pas terminé
# Augmenter le timeout dans le code si nécessaire
```

---

## 📝 Logs typiques d'un warmup réussi

```
[INFO] [curobo_gen_traj]: Starting warmup in background thread...
[INFO] [curobo_gen_traj]: Ready to generate trajectories
[INFO] [curobo_gen_traj]: Warmup thread: Starting warmup (estimated duration: 68.6s)
[INFO] [curobo_gen_traj]: Warmup thread: Starting CUDA warmup (this may take 30-60 seconds)...
[INFO] [curobo_gen_traj]: Warmup progress: 15.3% (5.0s / 68.6s)
[INFO] [curobo_gen_traj]: Warmup progress: 28.7% (10.0s / 68.6s)
[INFO] [curobo_gen_traj]: Warmup progress: 42.1% (15.0s / 68.6s)
[INFO] [curobo_gen_traj]: Warmup progress: 55.5% (20.0s / 68.6s)
[INFO] [curobo_gen_traj]: Warmup progress: 68.9% (25.0s / 68.6s)
[INFO] [curobo_gen_traj]: Warmup progress: 82.3% (30.0s / 68.6s)
[INFO] [curobo_gen_traj]: ✅ Warmup complete in 35.2s (estimated: 68.6s)
[INFO] [curobo_gen_traj]: ✅ Motion generation ready to use!
```

---

## 🎯 Résumé

| Fonctionnalité | Avant | Après |
|----------------|-------|-------|
| **Démarrage du node** | 🔴 Bloqué 30-60s | 🟢 Instantané |
| **RViz** | 🔴 Gelé | 🟢 Responsive |
| **Services** | 🔴 Indisponibles | 🟢 Attendent auto |
| **Feedback** | 🔴 Aucun | 🟢 Temps réel |
| **Topic de progression** | ❌ Non | ✅ `/warmup_progress` |
| **Estimation de durée** | ❌ Non | ✅ Basée sur config |

---

## 📚 Voir aussi

- [Architecture du système](architecture.md)
- [Interfaces ROS](ros_interfaces.md)
- [Configuration de motion generation](../tutorials/doosan_example.md)
