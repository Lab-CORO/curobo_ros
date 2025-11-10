# Tests d'Intégration CuRobo ROS

Ce dossier contient les tests d'intégration end-to-end (E2E) pour valider le fonctionnement complet du système curobo_ros.

## 📁 Structure

```
test/integration/
├── conftest.py                      # Fixtures pytest partagées
├── test_e2e_planning.py             # TEST_E2E_001: Pipeline planification
├── test_e2e_environment.py          # TEST_E2E_002: Environnement dynamique
├── test_e2e_perception.py           # TEST_E2E_003: Perception → Planification
├── test_e2e_strategies.py           # TEST_E2E_004: Stratégies robot
├── test_e2e_robustness.py           # TEST_E2E_005 & 006: Stress & Robustesse
└── README.md                        # Ce fichier
```

## 🎯 Scénarios de Test

### TEST_E2E_001: Pipeline Complet de Planification
- **Fichier**: `test_e2e_planning.py`
- **Objectif**: Valider la chaîne IK → Génération Trajectoire → Exécution
- **Tests**: 8 tests (a-h)
- **Durée estimée**: 5-10 minutes

**Tests inclus**:
- Génération simple de trajectoire
- Pipeline IK → Trajectoire
- Séquence pick-and-place complète
- Qualité de trajectoire
- Gestion pose inatteignable
- Mécanisme de retry

### TEST_E2E_002: Gestion d'Environnement Dynamique
- **Fichier**: `test_e2e_environment.py`
- **Objectif**: Valider ajout/suppression d'obstacles et replanification
- **Tests**: 9 tests (a-i)
- **Durée estimée**: 8-12 minutes

**Tests inclus**:
- Ajout/suppression obstacles
- Impact sur génération trajectoire
- Voxel grid reflète obstacles
- Distance de collision
- Replanification dynamique

### TEST_E2E_003: Pipeline Perception → Planification
- **Fichier**: `test_e2e_perception.py`
- **Objectif**: Valider perception caméra et impact sur planification
- **Tests**: 8 tests (a-h)
- **Durée estimée**: 10-15 minutes
- **Nécessite**: Rosbags de test

**Tests inclus**:
- Segmentation d'image depth
- Qualité segmentation robot
- Latence pipeline perception
- Point cloud → Voxel grid
- Obstacle perçu bloque trajectoire

### TEST_E2E_004: Stratégies Robot Multiples
- **Fichier**: `test_e2e_strategies.py`
- **Objectif**: Valider les différentes stratégies (emulator, ghost, doosan)
- **Tests**: 9 tests (a-i)
- **Durée estimée**: 5-8 minutes

**Tests inclus**:
- Get/Set stratégie
- Émulateur publie joint states
- Ghost publie trajectoire
- Interface Doosan
- Changement de stratégie

### TEST_E2E_005: Stress Test et Performance
- **Fichier**: `test_e2e_robustness.py` (classe TestE2EStressAndPerformance)
- **Objectif**: Valider stabilité sous charge
- **Tests**: 4 tests (a-d)
- **Durée estimée**: 15-20 minutes

**Tests inclus**:
- 100 requêtes IK concurrentes
- Batch IK 1000 poses
- Génération continue 100 trajectoires
- Ajout/suppression rapide obstacles

### TEST_E2E_006: Récupération d'Erreurs
- **Fichier**: `test_e2e_robustness.py` (classe TestE2EErrorRecovery)
- **Objectif**: Valider robustesse et recovery
- **Tests**: 7 tests (a-g)
- **Durée estimée**: 5-8 minutes

**Tests inclus**:
- Gestion pose invalide (NaN)
- Timeout handling
- Échecs répétés
- Récupération après erreur
- Requêtes concurrentes conflictuelles

## 🚀 Exécution des Tests

### Prérequis

```bash
# Installer les dépendances de test
pip install -r requirements_test.txt

# S'assurer que ROS 2 est sourcé
source /opt/ros/humble/setup.bash  # ou votre distro ROS 2

# Compiler le workspace
cd /path/to/workspace
colcon build

# Sourcer le workspace
source install/setup.bash
```

### Lancer les Tests

**Tous les tests d'intégration**:
```bash
pytest test/integration/ -v
```

**Un fichier de test spécifique**:
```bash
pytest test/integration/test_e2e_planning.py -v
```

**Un test spécifique**:
```bash
pytest test/integration/test_e2e_planning.py::TestE2EPlanning::test_simple_trajectory_generation -v
```

**Avec output détaillé**:
```bash
pytest test/integration/ -v -s
```

**Tests rapides uniquement (exclure @pytest.mark.slow)**:
```bash
pytest test/integration/ -v -m "not slow"
```

**Tests sans rosbags (skip tests qui nécessitent rosbags)**:
```bash
pytest test/integration/ -v -m "not requires_rosbag"
```

**Tests stress seulement**:
```bash
pytest test/integration/ -v -m stress
```

**Générer rapport HTML**:
```bash
pytest test/integration/ --html=test_report.html --self-contained-html
```

**Avec couverture de code**:
```bash
pytest test/integration/ --cov=curobo_ros --cov-report=html
```

## 🏷️ Marqueurs pytest

Les tests utilisent des marqueurs pour catégorisation:

- `@pytest.mark.integration` - Tous les tests d'intégration
- `@pytest.mark.e2e` - Tests end-to-end
- `@pytest.mark.slow` - Tests longs (>30s)
- `@pytest.mark.perception` - Tests de perception
- `@pytest.mark.strategies` - Tests de stratégies robot
- `@pytest.mark.stress` - Tests de stress/performance
- `@pytest.mark.robustness` - Tests de robustesse
- `@pytest.mark.requires_rosbag` - Nécessite rosbags
- `@pytest.mark.hardware` - Nécessite hardware réel

Configurer dans `pytest.ini` ou exécuter avec `-m`:

```bash
# Exclure tests lents
pytest -m "not slow"

# Seulement tests de perception
pytest -m perception

# Tests stress et robustness
pytest -m "stress or robustness"
```

## 📊 Métriques Collectées

Les tests collectent automatiquement des métriques de performance:

- **Temps de réponse**: Pour chaque service call
- **Latence**: Pipeline perception, IK→Trajectory, etc.
- **Fréquence**: Publication de topics
- **Utilisation mémoire**: Avant/après opérations
- **Taux de succès**: Pourcentage de requêtes réussies
- **Compteurs**: Nombre d'obstacles ajoutés, requêtes, etc.

Les métriques sont affichées à la fin de chaque test.

## 🎬 Rosbags Nécessaires

Certains tests nécessitent des rosbags. Voir `test/rosbags/README.md` pour détails.

**Rosbags P0 (essentiels)**:
- `test_depth_static_scene.db3` - Scène statique
- `test_robot_joint_states_variety.db3` - États joints variés
- `test_robot_trajectory_execution.db3` - Exécution trajectoire

**Télécharger ou enregistrer** selon instructions dans `test/rosbags/README.md`.

## 🔧 Configuration

### conftest.py

Le fichier `conftest.py` fournit des fixtures réutilisables:

**Fixtures ROS**:
- `rclpy_init` - Initialise ROS 2 context
- `ros_executor` - Executor multi-threaded
- `test_node` - Nœud de test pour service calls

**Fixtures Données**:
- `test_poses` - Poses prédéfinies (home, pick, place, unreachable)
- `test_obstacles` - Obstacles prédéfinis (box, cylinder, sphere)
- `test_config` - Configuration par défaut

**Fixtures Helpers**:
- `service_helper_factory` - Créer clients de service
- `topic_helper_factory` - Créer subscribers de topic
- `performance_metrics` - Collecter métriques

**Exemple d'utilisation**:
```python
def test_my_service(test_node, service_helper_factory, test_poses):
    # Créer client de service
    client = service_helper_factory(MyService, '/my_service')

    # Utiliser pose prédéfinie
    request = MyService.Request()
    request.pose = test_poses['pick']

    # Appeler service
    response = client.call(request, timeout=5.0)

    assert response.success
```

## 🐛 Debugging

**Verbose pytest output**:
```bash
pytest test/integration/ -vv -s --tb=long
```

**Arrêter au premier échec**:
```bash
pytest test/integration/ -x
```

**Entrer en debugger pdb au échec**:
```bash
pytest test/integration/ --pdb
```

**Logs ROS 2**:
```bash
# Augmenter niveau de logs
export RCUTILS_LOGGING_SEVERITY=DEBUG
pytest test/integration/ -v
```

**Run test isolé**:
```bash
pytest test/integration/test_e2e_planning.py::TestE2EPlanning::test_simple_trajectory_generation -v -s
```

## 📝 Écrire de Nouveaux Tests

### Template de Test

```python
import pytest
from curobo_msgs.srv import MyService

@pytest.mark.integration
@pytest.mark.e2e
class TestMyFeature:
    """Test suite for my feature"""

    @pytest.fixture(autouse=True)
    def setup(self, test_node, service_helper_factory):
        """Setup for all tests in this class"""
        self.node = test_node
        self.client = service_helper_factory(MyService, '/my_service')

    def test_basic_functionality(self):
        """
        TEST_XXX_YYY: Description

        Validations:
        - Item 1
        - Item 2
        """
        request = MyService.Request()
        # ... configure request

        response = self.client.call(request, timeout=5.0)

        assert response.success, f"Failed: {response.message}"

        print(f"\n✓ Test passed")
```

### Guidelines

1. **Nommage**: `test_e2e_<category>.py` pour fichiers, `TEST_E2E_00X` pour référence
2. **Docstrings**: Inclure description, validations attendues
3. **Assertions**: Toujours avec messages descriptifs
4. **Cleanup**: Utiliser fixtures pour setup/teardown
5. **Timeouts**: Toujours spécifier timeout sur service calls
6. **Prints**: Utiliser `print(f"\n✓ ...")` pour feedback visuel
7. **Marqueurs**: Ajouter marqueurs pytest appropriés

## 🔄 CI/CD

Les tests d'intégration peuvent être intégrés dans GitHub Actions:

```yaml
# .github/workflows/integration_tests.yml
name: Integration Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v3

      - name: Setup ROS 2
        uses: ros-tooling/setup-ros@v0.6
        with:
          required-ros-distributions: humble

      - name: Install dependencies
        run: |
          pip install -r requirements_test.txt

      - name: Build workspace
        run: |
          colcon build

      - name: Run integration tests
        run: |
          source install/setup.bash
          pytest test/integration/ -v -m "not hardware and not slow"
```

## 📈 Couverture de Test

Objectif: **>80%** couverture des interfaces ROS

**Vérifier couverture**:
```bash
pytest test/integration/ --cov=curobo_ros --cov-report=term-missing
```

**Générer rapport HTML**:
```bash
pytest test/integration/ --cov=curobo_ros --cov-report=html
firefox htmlcov/index.html
```

## 🆘 Support

- **Plan de test complet**: Voir `INTEGRATION_TEST_PLAN.md`
- **Rosbags**: Voir `test/rosbags/README.md`
- **Issues**: Créer issue GitHub avec tag `testing`

---

**Durée totale estimée**: ~50-70 minutes pour suite complète
**Tests**: ~45 tests d'intégration
**Couverture**: Services, Actions, Topics, Paramètres
