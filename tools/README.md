# Outils d'Analyse IK

Ce dossier contient des outils pour analyser et tester les solutions de cinématique inverse (IK) générées par CuRobo.

## Fichiers

### 1. `analyze_ik_solutions.py`

Script Python pour analyser l'unicité des solutions IK.

**Fonctionnalités:**
- Détecte les solutions doublons (convergence multiple vers même solution)
- Compte le nombre de solutions uniques
- Calcule des métriques de diversité
- Affiche un rapport détaillé

**Usage:**

```bash
# Mode test avec données simulées
python3 tools/analyze_ik_solutions.py --test --num_seeds 20

# Personnaliser le seuil d'unicité
python3 tools/analyze_ik_solutions.py --test --num_seeds 50 --seuil 0.02

# Aide
python3 tools/analyze_ik_solutions.py --help
```

**Paramètres:**
- `--num_seeds`: Nombre de graines IK (défaut: 20)
- `--seuil`: Seuil d'unicité en radians (défaut: 0.01 rad ≈ 0.57°)
- `--test`: Mode test avec solutions simulées

**Note:** Pour analyser de vraies solutions CuRobo, intégrer le code dans le nœud ROS `/curobo_ik`.

### 2. `demo_unicite_solutions.txt`

Exemple de sortie du script d'analyse montrant:
- Comment 20 graines peuvent donner seulement 4 solutions uniques
- Distribution de la convergence multiple
- Solutions potentiellement manquantes
- Recommandations pour améliorer la couverture

## Contexte

### Problème

CuRobo avec `num_seeds=20` génère 20 solutions IK, mais:
- ❌ Ce ne sont pas nécessairement 20 solutions **différentes**
- ❌ On n'a pas nécessairement **toutes** les solutions possibles

**Raison:** CuRobo utilise optimisation numérique avec graines aléatoires. Plusieurs graines peuvent converger vers le même minimum local.

### Solutions Théoriques

Pour un robot 6-DOF:
- **Poignet sphérique:** Jusqu'à **8 solutions** distinctes
- **Configuration générale:** Jusqu'à **16 solutions** distinctes

### Exemple Typique

Avec `num_seeds=20` pour une pose générique:
- 20 graines lancées
- 18 convergent (90% de réussite)
- **4 solutions uniques** trouvées (22% d'unicité, 78% de doublons)
- **4 solutions manquantes** (non trouvées par aucune graine)

## Recommandations

### Pour Avoir Plus de Solutions Uniques

1. **Augmenter num_seeds**
   ```python
   # config_wrapper_motion.py:189
   num_seeds=100  # Au lieu de 20
   ```
   - ✅ Plus de chances de trouver toutes les solutions
   - ❌ Temps de calcul proportionnel

2. **Filtrer les doublons**
   - Utiliser `IKSolutionAnalyzer.filtrer_solutions_uniques()`
   - Ne retourner que les solutions distinctes

3. **Méthode hybride (idéal)**
   - IK analytique pour garantir toutes les solutions
   - CuRobo pour vérifier les collisions
   - Nécessite implémentation spécifique au robot

### Tableau de Probabilité

| num_seeds | Prob. toutes solutions | Temps GPU |
|-----------|----------------------|-----------|
| 20        | ~50%                 | ~10 ms    |
| 50        | ~80%                 | ~25 ms    |
| 100       | ~95%                 | ~50 ms    |
| 200       | ~99%                 | ~100 ms   |

## Documents Connexes

- `../ANALYSE_UNICITE_SOLUTIONS_IK.md` - Analyse théorique détaillée
- `../EVALUATION_IK_MULTIPLES_SOLUTIONS.md` - Évaluation de faisabilité

## Auteur

Claude - 2025-11-13
