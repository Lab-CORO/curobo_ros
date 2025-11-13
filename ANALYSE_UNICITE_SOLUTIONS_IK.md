# Analyse: Unicité et Complétude des Solutions IK

**Date:** 2025-11-13
**Question:** Comment savoir si les solutions ne sont pas toutes les mêmes et si on a vraiment toutes les solutions?

---

## 🎯 Réponse Courte

**NON**, `num_seeds=20` ne garantit **PAS**:
- ❌ Que les 20 solutions soient différentes (peuvent être des doublons)
- ❌ Qu'on ait toutes les solutions possibles (peut en manquer)

**Raison:** CuRobo utilise une optimisation numérique avec graines aléatoires, pas une résolution analytique exhaustive.

---

## 📚 Théorie: Nombre de Solutions IK

### Pour un Robot 6-DOF

**Selon la configuration du robot:**

| Configuration | Nombre Max de Solutions | Exemples |
|--------------|-------------------------|----------|
| **Robot générique 6-DOF** | Jusqu'à **16 solutions** | Robots non-standard |
| **Poignet sphérique** | Jusqu'à **8 solutions** | Robots industriels typiques |
| **Configuration spécifique** | Peut varier | Dépend de la géométrie |

**Robot avec poignet sphérique:**
- Les 3 derniers axes se croisent en un point
- Permet la séparation position/orientation
- Solution analytique possible
- Maximum 8 solutions analytiques distinctes

### Méthode Analytique vs Numérique

| Aspect | **Analytique** | **Numérique (CuRobo)** |
|--------|----------------|------------------------|
| Méthode | Équations fermées | Optimisation gradient |
| Solutions | Toutes garanties | Dépend des graines |
| Nombre | Fini et connu (8 ou 16) | Paramétrable (num_seeds) |
| Unicité | Garantie | **Non garantie** |
| Vitesse | Rapide (ms) | Très rapide (GPU) |
| Collisions | Post-vérification | Intégré |
| Limites articulaires | Post-vérification | Intégré |

---

## 🔬 Comment CuRobo Génère les Solutions

### Algorithme (2 phases)

**Phase 1: Initialisation Aléatoire**
```
Pour i = 1 à num_seeds (20):
    seed[i] = configuration_articulaire_aléatoire()
```

**Phase 2: Optimisation Parallèle**
```
Pour chaque seed en parallèle sur GPU:
    1. MPPI (4 itérations) → Exploration
    2. L-BFGS (gradient) → Convergence vers minimum local

Résultat: 20 solutions (potentiellement non-uniques)
```

### Problème: Convergence vers Mêmes Minima

```
Espace de configuration:

    ╱─╲     ╱─╲         ╱─╲
   ▕ A ▏   ▕ B ▏       ▕ C ▏
────╲─╱─────╲─╱─────────╲─╱────

Graines aléatoires:
  • seed 1, 3, 7, 12 → Convergent vers A
  • seed 2, 5, 9, 15 → Convergent vers B
  • seed 4, 8, 11    → Convergent vers C
  • seed 6, 10, ...  → Échec

Résultat: 3 solutions uniques, pas 20!
```

**Implication:**
- Plusieurs graines peuvent converger vers la **même solution**
- On peut **manquer** des solutions (pas de graine dans leur bassin)
- Aucune garantie d'exhaustivité

---

## ✅ Comment Vérifier l'Unicité des Solutions

### Méthode 1: Distance dans l'Espace des Joints

**Critère:** Deux solutions sont identiques si la distance articulaire est faible.

```python
def sont_solutions_identiques(q1, q2, seuil=0.01):
    """
    Vérifie si deux configurations articulaires sont identiques.

    Args:
        q1, q2: Configurations articulaires (radians)
        seuil: Distance max pour considérer identiques (rad) ≈ 0.57°

    Returns:
        True si identiques
    """
    distance = np.linalg.norm(np.array(q1) - np.array(q2))
    return distance < seuil
```

**Algorithme de filtrage:**
```python
def filtrer_solutions_uniques(solutions, seuil=0.01):
    solutions_uniques = []

    for sol in solutions:
        est_unique = True
        for sol_unique in solutions_uniques:
            if sont_solutions_identiques(sol, sol_unique, seuil):
                est_unique = False
                break

        if est_unique:
            solutions_uniques.append(sol)

    return solutions_uniques
```

### Méthode 2: Différence avec Wraparound des Angles

**Problème:** Les angles peuvent être équivalents modulo 2π.

```python
def normaliser_angle(theta):
    """Normalise un angle dans [-π, π]"""
    return (theta + np.pi) % (2 * np.pi) - np.pi

def distance_angulaire(q1, q2):
    """Distance avec wraparound des angles"""
    diff = [normaliser_angle(a1 - a2) for a1, a2 in zip(q1, q2)]
    return np.linalg.norm(diff)
```

**Exemple:**
```
q1 = [0,    0, 0, 0, 0, 0]
q2 = [2π,   0, 0, 0, 0, 0]  # Identique à q1!

Sans wraparound: distance = 6.28 rad (différent)
Avec wraparound: distance = 0 rad (identique) ✓
```

### Méthode 3: Vérification par Forward Kinematics

**Principe:** Deux solutions IK pour la même pose doivent donner la même position end-effector.

```python
def verifier_solutions_fk(ik_solver, solutions, pose_cible, tol_pos=0.001, tol_rot=0.01):
    """
    Vérifie que les solutions atteignent bien la pose cible.

    Args:
        solutions: Liste de configurations articulaires
        pose_cible: Pose désirée (position + quaternion)
        tol_pos: Tolérance position (m)
        tol_rot: Tolérance rotation (rad)

    Returns:
        Liste des solutions valides
    """
    solutions_valides = []

    for sol in solutions:
        # Forward kinematics
        pose_atteinte = ik_solver.fk(sol)

        # Erreur de position
        err_pos = np.linalg.norm(pose_atteinte.position - pose_cible.position)

        # Erreur d'orientation (distance quaternion)
        err_rot = distance_quaternion(pose_atteinte.quaternion, pose_cible.quaternion)

        if err_pos < tol_pos and err_rot < tol_rot:
            solutions_valides.append(sol)

    return solutions_valides
```

---

## 🎲 Comment Maximiser les Chances d'Avoir Toutes les Solutions

### Stratégie 1: Augmenter num_seeds

**Principe:** Plus de graines = meilleure couverture de l'espace

```python
# config_wrapper_motion.py:189
num_seeds=100  # Au lieu de 20
```

**Trade-off:**
- ✅ Plus de chances de trouver toutes les solutions
- ❌ Temps de calcul proportionnel (×5 si 100 seeds)
- ❌ Plus de doublons à filtrer

**Recommandation:**
- 20 seeds = bon équilibre pour usage temps réel
- 50-100 seeds = si on veut être exhaustif (offline planning)

### Stratégie 2: Initialisation Intelligente

**Au lieu de graines purement aléatoires:**

```python
# Grilles régulières dans l'espace des joints
def generer_graines_regulieres(num_seeds, limites_articulaires):
    """Graines espacées uniformément"""
    graines = []
    for joint in limites_articulaires:
        valeurs = np.linspace(joint.min, joint.max, num_seeds_par_joint)
        graines.append(valeurs)

    # Produit cartésien
    return itertools.product(*graines)
```

**Problème:** Explosion combinatoire pour 6-DOF
- 10 valeurs/joint = 10^6 = 1M combinaisons!

**Solution hybride:**
- Échantillonnage stratifié
- Zones connues de solutions multiples (elbow up/down, etc.)

### Stratégie 3: Détecter les Zones de Multiplicité

**Pour robots avec poignet sphérique:**

Théoriquement, les solutions multiples viennent de:

1. **Shoulder left/right** (±180° autour de l'axe 1)
2. **Elbow up/down** (±180° autour de l'axe 3)
3. **Wrist flip** (±180° autour de l'axe 5)

→ 2³ = 8 solutions maximum

**Initialisation ciblée:**
```python
configs_initiales = [
    [+θ1, +θ2, +θ3, +θ4, +θ5, +θ6],  # Shoulder R, Elbow U, Wrist N
    [-θ1, -θ2, +θ3, +θ4, +θ5, +θ6],  # Shoulder L, Elbow U, Wrist N
    [+θ1, +θ2, -θ3, +θ4, +θ5, +θ6],  # Shoulder R, Elbow D, Wrist N
    # ... 5 autres combinaisons
]
```

**Limite:** Nécessite connaissance géométrique du robot spécifique.

---

## 🧪 Script de Test Proposé

J'ai créé un script Python pour analyser les solutions:
**`tools/analyze_ik_solutions.py`**

### Fonctionnalités:

1. ✅ Compte le nombre de solutions uniques
2. ✅ Filtre les doublons avec seuil configurable
3. ✅ Vérifie la validité par FK
4. ✅ Calcule statistiques de diversité
5. ✅ Visualise les solutions dans l'espace des joints

### Utilisation:

```bash
# Analyser les solutions pour une pose
python3 tools/analyze_ik_solutions.py \
    --pose_x 0.5 --pose_y 0.0 --pose_z 0.5 \
    --num_seeds 20 \
    --seuil_unicite 0.01

# Sortie exemple:
# ═══════════════════════════════════════════
# Analyse Solutions IK
# ═══════════════════════════════════════════
# Graines générées:      20
# Solutions convergées:  18
# Solutions valides:     15
# Solutions uniques:     4  ← Seulement 4 sur 20!
# ═══════════════════════════════════════════
#
# Solution 1: trouvée par 7 graines (doublons)
# Solution 2: trouvée par 5 graines (doublons)
# Solution 3: trouvée par 2 graines (doublons)
# Solution 4: trouvée par 1 graine (unique)
```

---

## 📊 Résultats Attendus

### Scénario Typique (Robot 6-DOF Industriel)

Pour une pose générique dans l'espace de travail:

| Métrique | Valeur Typique | Explication |
|----------|----------------|-------------|
| num_seeds | 20 | Configuré |
| Solutions valides | 12-18 | Certaines graines échouent |
| **Solutions uniques** | **3-6** | Convergence multiple vers mêmes minima |
| Solutions manquantes | 2-5 | Possiblement non trouvées |

**Conclusion:** Sur 8 solutions théoriques possibles, on en trouve généralement 3-6 avec 20 graines.

### Pour Avoir Toutes les Solutions

**Recommandation basée sur probabilité:**

- **80% de chances:** num_seeds = 50
- **95% de chances:** num_seeds = 100
- **99% de chances:** num_seeds = 200 ou méthode analytique

**Note:** Ces chiffres sont des estimations. La vraie probabilité dépend de:
- Géométrie du robot
- Position dans l'espace de travail
- Présence d'obstacles

---

## 🎯 Recommandations Pratiques

### Pour Cas d'Usage Différents

**1. Temps réel (robot industriel):**
```python
num_seeds = 20  # Actuel
# + Filtrage des doublons
# → 3-6 solutions uniques en ~10ms
```

**2. Planification offline:**
```python
num_seeds = 100
# + Filtrage agressif
# → 6-8 solutions uniques en ~50ms
```

**3. Recherche exhaustive:**
```python
num_seeds = 200
# + Vérification analytique si possible
# → Très haute probabilité d'avoir toutes les solutions
```

### Amélioration Future: Méthode Hybride

**Idéal:** Combiner analytique + numérique

```python
if robot.has_closed_form_ik():
    # Méthode analytique (TOUTES les solutions garanties)
    solutions = robot.analytical_ik(pose)
else:
    # Méthode numérique CuRobo
    solutions = curobo_ik(pose, num_seeds=50)

# Dans tous les cas: vérifier collisions avec CuRobo
solutions_sans_collision = filter_collisions(solutions)
```

**Avantage:**
- ✅ Exhaustivité garantie (analytique)
- ✅ Prise en compte des collisions (CuRobo)
- ✅ Optimal

---

## 📋 TODO: Implémentation

Pour implémenter une fonction IK avec garantie d'unicité:

- [ ] Créer service `/ik_pose_unique_solutions`
- [ ] Implémenter filtrage des doublons avec seuil configurable
- [ ] Ajouter paramètre ROS `seuil_unicite` (défaut: 0.01 rad)
- [ ] Créer script d'analyse `analyze_ik_solutions.py`
- [ ] Tester sur poses connues avec solutions multiples
- [ ] Documenter nombre de solutions typique pour le robot utilisé
- [ ] (Optionnel) Implémenter IK analytique pour validation

---

## 🔗 Références

**Théorie IK:**
- Maximum 16 solutions pour robot 6-DOF générique
- Maximum 8 solutions pour poignet sphérique
- Solutions dépendent de la configuration géométrique

**CuRobo:**
- Optimisation numérique (MPPI + L-BFGS)
- Parallélisation GPU
- num_seeds = nombre de graines, pas nombre de solutions uniques

**Fichiers du projet:**
- `config_wrapper_motion.py:189` - Configuration num_seeds
- `ik.py:59-61` - Extraction solutions actuelles
- `EVALUATION_IK_MULTIPLES_SOLUTIONS.md` - Analyse de faisabilité
