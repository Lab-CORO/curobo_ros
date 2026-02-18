# Release Checklist - Branch MPC

Analyse de la branche `mpc` pour préparer un release propre et utilisable.

**Date**: 2025-12-04
**Branche analysée**: `mpc` (commit: 4540308)
**Branche documentation**: `claude/reorganize-documentation-01MyaFbvPmXjdMV4CJNrq1Tu`

---

## ✅ Fonctionnalités Implémentées

### 1. Architecture Unified Planner ✅
- **Statut**: Implémenté et fonctionnel
- **Fichiers**:
  - `curobo_ros/planners/trajectory_planner.py` (classe abstraite)
  - `curobo_ros/planners/planner_factory.py` (factory + manager)
  - `curobo_ros/planners/single_planner.py` (classe de base)

**Caractéristiques**:
- Factory pattern pour créer dynamiquement les planners
- Manager pour cache et switching entre planners
- Interface abstraite `TrajectoryPlanner` avec modes d'exécution
- Support pour enregistrer des planners custom

### 2. Planners Disponibles ✅
- **ClassicPlanner** (`classic_planner.py`) - Planning one-shot
- **MPCPlanner** (`mpc_planner.py`) - Model Predictive Control
- **MultiPointPlanner** (`multi_point_planner.py`) - Trajectoires multi-waypoints

### 3. Unified Planner Node ✅
- **Fichier**: `curobo_ros/core/unified_planner_node.py`
- Intègre factory et manager
- Services pour changer de planner dynamiquement
- Support des contraintes (start pose, goal constraints)

### 4. Nouvelles Fonctionnalités
- **Multi-point trajectories**: Planification à travers plusieurs waypoints
- **Contraintes**: Support des contraintes de pose de départ et d'arrivée
- **Configuration caméras**: Support pour extrinsèques/intrinsèques manuels
- **Paramètres par défaut**: Valeurs par défaut pour `camera_file` et `robot_file`

---

## ⚠️ PROBLÈMES CRITIQUES À RÉSOUDRE

### 1. 🔴 TODOs dans le Code (HAUTE PRIORITÉ)

#### Code Principal:
```python
# curobo_ros/core/mpc.py:38
self.robot_context = RobotContext(self, 0.03) # TODO issue dt

# curobo_ros/core/generate_trajectory.py:45
self.robot_context = RobotContext(self, 0.03) # TODO issue dt

# curobo_ros/core/generate_trajectory.py:128
# Currently there is not verifications (TODO)

# curobo_ros/core/config_wrapper_motion.py:67
# TODO Have these values be able to be overwritten in a launch file

# curobo_ros/core/config_wrapper_motion.py:70
self.collision_cache = {'obb': 100, 'blox': 10} # TODO: make this configurable

# curobo_ros/core/config_wrapper_motion.py:128
# TODO issue when modif acceleration scale

# curobo_ros/core/config_wrapper_motion.py:200
# TODO Should use lifecycle
```

**Actions requises**:
- ✅ Résoudre le problème du `dt` (timestep) hardcodé
- ✅ Ajouter des vérifications avant l'exécution de trajectoires
- ✅ Rendre `collision_cache` configurable via ROS params
- ✅ Documenter ou résoudre le problème d'`acceleration_scale`
- ⚠️ Considérer lifecycle nodes (optionnel pour v1.0)

### 2. 🔴 Documentation Non Synchronisée (CRITIQUE)

**Problème**: La branche `mpc` utilise l'ancienne structure de documentation

**Fichiers obsolètes dans mpc**:
- Liens vers `doc/concepts/introduction.md` (maintenant `getting-started/introduction.md`)
- Liens vers `doc/getting-started/installation.md` (maintenant `getting-started/installation.md`)
- Liens vers `doc/tutorials/1_first_trajectory.md` (maintenant `01-first-trajectory.md`)
- Référence à `doc/concepts/warmup_async.md` (supprimé)

**Actions requises**:
1. ✅ **CRITIQUE**: Merger la branche `claude/reorganize-documentation-01MyaFbvPmXjdMV4CJNrq1Tu` dans `mpc`
2. ✅ Mettre à jour tous les liens dans le README.md de mpc
3. ✅ S'assurer que la documentation reflète les fonctionnalités réelles

### 3. 🟡 Tests Manquants (MOYENNE PRIORITÉ)

**Tests existants**:
- `tests/test_basic.py` - Tests de base
- `tests/test_fk.py` - Tests FK
- `tests/test_pep257.py`, `test_flake8.py`, `test_copyright.py` - Linting
- `test_strategy_pattern.py` - Tests pattern
- `tests/curobo_test.py` - Tests cuRobo

**Tests manquants**:
- ❌ Tests pour `ClassicPlanner`
- ❌ Tests pour `MPCPlanner`
- ❌ Tests pour `MultiPointPlanner`
- ❌ Tests pour `PlannerFactory` et `PlannerManager`
- ❌ Tests d'intégration pour unified_planner_node
- ❌ Tests pour les contraintes
- ❌ Tests pour multi-point trajectories

**Actions requises**:
- ⚠️ Créer tests unitaires pour chaque planner
- ⚠️ Créer tests d'intégration ROS2
- ⚠️ Ajouter CI/CD pour exécuter les tests automatiquement

### 4. 🟡 Exemples et Tutoriels (MOYENNE PRIORITÉ)

**Exemple existant**:
- `examples/planner_usage_example.py` - Exemple basique

**Exemples manquants**:
- ❌ Exemple d'utilisation de `MultiPointPlanner`
- ❌ Exemple de switching entre planners
- ❌ Exemple d'utilisation des contraintes
- ❌ Exemple complet avec caméra
- ❌ Exemple d'intégration dans un node custom

**Tutoriels à mettre à jour**:
- ⚠️ Tutorial 5 (MPC) - Besoin de validation avec code réel
- ⚠️ Créer tutorial pour multi-point planning
- ⚠️ Créer tutorial pour dynamic planner switching

### 5. 🟡 Configuration et Paramètres (MOYENNE PRIORITÉ)

**Problèmes**:
- Certains paramètres sont hardcodés
- Pas de validation des paramètres
- Documentation des paramètres incomplète

**Actions requises**:
- ✅ Créer fichier de configuration YAML complet avec valeurs par défaut
- ✅ Ajouter validation des paramètres au démarrage
- ✅ Documenter tous les paramètres disponibles
- ✅ Créer exemples de configs pour différents use cases

### 6. 🟢 Messages/Services (BASSE PRIORITÉ)

**Services existants**:
- `srv/SetRobotStrategy.srv` - Change robot strategy

**Services potentiellement manquants**:
- Service pour lister les planners disponibles?
- Service pour obtenir le planner actuel?
- Service pour obtenir les capacités du planner?

**Actions**:
- ⚠️ Vérifier si tous les services nécessaires sont présents
- ⚠️ Documenter tous les services dans `doc/concepts/ros-interfaces.md`

---

## 📋 CHECKLIST RELEASE v1.0

### Phase 1: Corrections Critiques (OBLIGATOIRE)

- [ ] **1.1 Code TODOs**
  - [ ] Résoudre tous les TODOs critiques (dt, verifications, etc.)
  - [ ] Rendre collision_cache configurable
  - [ ] Tester avec différentes valeurs d'acceleration_scale

- [ ] **1.2 Documentation**
  - [ ] Merger branche documentation dans mpc
  - [ ] Mettre à jour README.md avec nouveaux liens
  - [ ] Vérifier que tous les liens fonctionnent
  - [ ] Mettre à jour doc/concepts/ros-interfaces.md avec nouveaux services

- [ ] **1.3 Configuration**
  - [ ] Créer fichier de config complet avec commentaires
  - [ ] Ajouter validation des paramètres
  - [ ] Documenter tous les paramètres

### Phase 2: Tests et Validation (RECOMMANDÉ)

- [ ] **2.1 Tests Unitaires**
  - [ ] Tests pour PlannerFactory
  - [ ] Tests pour chaque planner (Classic, MPC, MultiPoint)
  - [ ] Tests pour contraintes

- [ ] **2.2 Tests d'Intégration**
  - [ ] Test de switching entre planners
  - [ ] Test multi-point trajectories
  - [ ] Test avec caméra

- [ ] **2.3 Validation Manuelle**
  - [ ] Tester classic planner avec robot réel
  - [ ] Tester MPC planner (si implémenté)
  - [ ] Tester multi-point planner
  - [ ] Tester dynamic switching

### Phase 3: Documentation et Exemples (RECOMMANDÉ)

- [ ] **3.1 Exemples**
  - [ ] Exemple multi-point planning
  - [ ] Exemple planner switching
  - [ ] Exemple contraintes

- [ ] **3.2 Tutoriels**
  - [ ] Valider Tutorial 5 (MPC)
  - [ ] Créer tutorial multi-point
  - [ ] Créer tutorial dynamic switching

- [ ] **3.3 Documentation API**
  - [ ] Documenter tous les planners
  - [ ] Documenter factory et manager
  - [ ] Documenter services ROS

### Phase 4: Polish Final (OPTIONNEL)

- [ ] **4.1 Code Quality**
  - [ ] Linting complet (flake8, black)
  - [ ] Type hints partout
  - [ ] Docstrings complètes

- [ ] **4.2 Performance**
  - [ ] Benchmarks de performance
  - [ ] Profiling GPU/CPU
  - [ ] Optimisations si nécessaire

- [ ] **4.3 CI/CD**
  - [ ] GitHub Actions pour tests
  - [ ] Docker images sur registry
  - [ ] Documentation auto-générée

---

## 🎯 RECOMMANDATIONS PRIORITAIRES

### Pour un Release Minimal (v1.0-beta)

**DOIT être fait** (estimé: 2-3 jours):
1. ✅ Résoudre TODOs critiques dans le code
2. ✅ Merger documentation et corriger les liens
3. ✅ Créer configuration complète avec validation
4. ✅ Tester manuellement tous les planners
5. ✅ Mettre à jour README avec features actuelles

**DEVRAIT être fait** (estimé: 3-5 jours):
6. ⚠️ Créer tests unitaires de base
7. ⚠️ Créer 2-3 exemples clairs
8. ⚠️ Valider tutorials existants
9. ⚠️ Documenter API dans doc/concepts/

**PEUT être fait plus tard** (v1.1):
10. Lifecycle nodes
11. CI/CD complet
12. Benchmarks performance
13. Tutoriels avancés

### Pour un Release Complet (v1.0)

Tout dans Release Minimal + Phase 2 (Tests) + Phase 3 (Doc/Exemples)

**Estimé**: 1-2 semaines de travail

---

## 📊 RÉSUMÉ

### ✅ Points Forts
- Architecture propre et extensible
- Factory pattern bien implémenté
- Code modulaire et réutilisable
- Support multi-planners fonctionnel
- Nouvelles features (multi-point, contraintes)

### ⚠️ Points à Améliorer
- TODOs dans le code à résoudre
- Documentation pas synchronisée
- Tests insuffisants
- Exemples manquants
- Validation des paramètres à ajouter

### 🎯 Action Immédiate
**Pour sortir un release rapidement**:
1. Résoudre les TODOs critiques (1 jour)
2. Merger + synchroniser documentation (1 jour)
3. Tests manuels complets (1 jour)
4. Release v1.0-beta

**Total**: 3 jours de travail pour un beta utilisable

---

## 🔗 Ressources

- **Branche MPC**: `mpc` (commit 4540308)
- **Branche Doc**: `claude/reorganize-documentation-01MyaFbvPmXjdMV4CJNrq1Tu` (commit 61965e4)
- **README**: Besoin de mise à jour des liens
- **Documentation**: doc/ (nouvelle structure)

---

**Conclusion**: La branche `mpc` est techniquement solide mais nécessite:
1. Résolution des TODOs (critique)
2. Synchronisation documentation (critique)
3. Tests et validation (recommandé)

Un release beta peut être fait en 3 jours. Un release v1.0 complet nécessite 1-2 semaines.
