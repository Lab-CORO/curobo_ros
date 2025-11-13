#!/usr/bin/env python3
"""
Script d'analyse de l'unicité des solutions IK de CuRobo

Objectif: Vérifier combien de solutions UNIQUES sont trouvées parmi les num_seeds solutions générées.

Usage:
    python3 analyze_ik_solutions.py --num_seeds 20 --seuil 0.01

Auteur: Claude
Date: 2025-11-13
"""

import numpy as np
import argparse
from typing import List, Tuple
import sys


class IKSolutionAnalyzer:
    """Analyse l'unicité et la qualité des solutions IK."""

    def __init__(self, seuil_unicite: float = 0.01):
        """
        Args:
            seuil_unicite: Distance max (rad) pour considérer deux solutions identiques
                          Défaut: 0.01 rad ≈ 0.57°
        """
        self.seuil = seuil_unicite

    @staticmethod
    def normaliser_angle(theta: float) -> float:
        """Normalise un angle dans [-π, π]"""
        return (theta + np.pi) % (2 * np.pi) - np.pi

    def distance_angulaire(self, q1: np.ndarray, q2: np.ndarray) -> float:
        """
        Calcule la distance entre deux configurations avec wraparound des angles.

        Args:
            q1, q2: Configurations articulaires (radians)

        Returns:
            Distance L2 dans l'espace des joints (avec normalisation angulaire)
        """
        # Normaliser les différences angulaires
        diff = np.array([self.normaliser_angle(a1 - a2) for a1, a2 in zip(q1, q2)])
        return np.linalg.norm(diff)

    def sont_identiques(self, q1: np.ndarray, q2: np.ndarray) -> bool:
        """Vérifie si deux configurations sont identiques selon le seuil."""
        return self.distance_angulaire(q1, q2) < self.seuil

    def filtrer_solutions_uniques(
        self, solutions: np.ndarray, success_flags: np.ndarray = None
    ) -> Tuple[List[np.ndarray], List[int]]:
        """
        Filtre les solutions pour ne garder que les uniques.

        Args:
            solutions: Array de forme [num_seeds, num_joints]
            success_flags: Array de forme [num_seeds] indiquant les solutions valides

        Returns:
            (solutions_uniques, compteur_occurrences)
            - solutions_uniques: Liste des configurations uniques
            - compteur_occurrences: Combien de graines ont trouvé chaque solution
        """
        solutions_uniques = []
        compteur_occurrences = []

        # Filtrer seulement les solutions valides si flags fournis
        if success_flags is not None:
            indices_valides = np.where(success_flags)[0]
            solutions_a_analyser = solutions[indices_valides]
        else:
            solutions_a_analyser = solutions

        for sol in solutions_a_analyser:
            # Vérifier si cette solution est déjà dans la liste des uniques
            est_nouvelle = True
            for idx, sol_unique in enumerate(solutions_uniques):
                if self.sont_identiques(sol, sol_unique):
                    compteur_occurrences[idx] += 1
                    est_nouvelle = False
                    break

            if est_nouvelle:
                solutions_uniques.append(sol)
                compteur_occurrences.append(1)

        return solutions_uniques, compteur_occurrences

    def analyser_diversite(
        self, solutions_uniques: List[np.ndarray]
    ) -> dict:
        """
        Calcule des métriques de diversité des solutions.

        Returns:
            dict avec:
            - distance_min: Plus petite distance entre deux solutions
            - distance_max: Plus grande distance entre deux solutions
            - distance_moyenne: Distance moyenne entre toutes paires
        """
        if len(solutions_uniques) < 2:
            return {
                "distance_min": 0.0,
                "distance_max": 0.0,
                "distance_moyenne": 0.0,
            }

        distances = []
        for i, sol1 in enumerate(solutions_uniques):
            for sol2 in solutions_uniques[i + 1 :]:
                distances.append(self.distance_angulaire(sol1, sol2))

        return {
            "distance_min": np.min(distances),
            "distance_max": np.max(distances),
            "distance_moyenne": np.mean(distances),
        }


def generer_solutions_test(num_seeds: int, num_joints: int = 6) -> Tuple[np.ndarray, np.ndarray]:
    """
    Génère des solutions de test pour démonstration.

    Simule le comportement de CuRobo:
    - Plusieurs graines convergent vers les mêmes solutions
    - Certaines graines échouent
    """
    # Définir 4 solutions distinctes "cibles"
    solutions_cibles = np.array([
        [0.0, -0.5, 1.2, 0.0, 0.8, 0.0],      # Solution 1 (Elbow up)
        [0.0, -0.5, -1.2, 0.0, -0.8, 0.0],    # Solution 2 (Elbow down)
        [3.14, 0.5, 1.2, 3.14, 0.8, 3.14],    # Solution 3 (Shoulder flip)
        [-1.5, -1.0, 0.5, 1.0, 1.5, -0.5],    # Solution 4 (Alternative)
    ])

    solutions = np.zeros((num_seeds, num_joints))
    success_flags = np.zeros(num_seeds, dtype=bool)

    # Distribuer les graines vers différentes solutions avec bruit
    for i in range(num_seeds):
        if i < num_seeds * 0.9:  # 90% de réussite
            # Choisir une solution cible (biaisé vers les premières)
            proba = [0.35, 0.30, 0.20, 0.15]  # Plus de graines vers solutions 1-2
            idx_cible = np.random.choice(len(solutions_cibles), p=proba)

            # Ajouter un petit bruit (convergence imparfaite)
            bruit = np.random.normal(0, 0.005, num_joints)  # ~0.3°
            solutions[i] = solutions_cibles[idx_cible] + bruit

            success_flags[i] = True
        else:
            # 10% d'échec - solution aléatoire invalide
            solutions[i] = np.random.uniform(-3.14, 3.14, num_joints)
            success_flags[i] = False

    return solutions, success_flags


def afficher_rapport(
    num_seeds: int,
    solutions: np.ndarray,
    success_flags: np.ndarray,
    solutions_uniques: List[np.ndarray],
    compteur: List[int],
    metriques: dict,
    seuil: float,
):
    """Affiche un rapport formaté de l'analyse."""

    print("\n" + "=" * 70)
    print(" ANALYSE DES SOLUTIONS IK - UNICITÉ ET DIVERSITÉ")
    print("=" * 70)

    print(f"\n📊 STATISTIQUES GLOBALES")
    print(f"{'─' * 70}")
    print(f"  Graines générées (num_seeds):     {num_seeds}")
    print(f"  Solutions convergées:              {len(solutions)}")
    print(f"  Solutions valides (success=True):  {np.sum(success_flags)}")
    print(f"  Solutions UNIQUES trouvées:        {len(solutions_uniques)} ⭐")
    print(f"  Taux de convergence:               {np.sum(success_flags)/num_seeds*100:.1f}%")
    print(f"  Taux d'unicité:                    {len(solutions_uniques)/np.sum(success_flags)*100:.1f}%")

    print(f"\n⚙️  PARAMÈTRES")
    print(f"{'─' * 70}")
    print(f"  Seuil d'unicité:                   {seuil:.4f} rad ({np.degrees(seuil):.2f}°)")

    if len(solutions_uniques) > 1:
        print(f"\n📏 DIVERSITÉ DES SOLUTIONS")
        print(f"{'─' * 70}")
        print(f"  Distance minimale entre solutions: {metriques['distance_min']:.4f} rad "
              f"({np.degrees(metriques['distance_min']):.2f}°)")
        print(f"  Distance maximale entre solutions: {metriques['distance_max']:.4f} rad "
              f"({np.degrees(metriques['distance_max']):.2f}°)")
        print(f"  Distance moyenne entre solutions:  {metriques['distance_moyenne']:.4f} rad "
              f"({np.degrees(metriques['distance_moyenne']):.2f}°)")

    print(f"\n🎯 DÉTAIL DES SOLUTIONS UNIQUES")
    print(f"{'─' * 70}")

    for idx, (sol, count) in enumerate(zip(solutions_uniques, compteur)):
        pourcentage = (count / np.sum(success_flags)) * 100
        print(f"\n  Solution #{idx + 1} (trouvée par {count} graine(s) = {pourcentage:.1f}%)")
        print(f"    Joints (rad): {np.array2string(sol, precision=3, suppress_small=True)}")
        print(f"    Joints (deg): {np.array2string(np.degrees(sol), precision=1, suppress_small=True)}")

        if count > 1:
            print(f"    ⚠️  ATTENTION: {count - 1} doublons détectés (convergence multiple)")

    print(f"\n{'=' * 70}")

    # Avertissement si peu de solutions
    if len(solutions_uniques) < 4:
        print(f"\n⚠️  AVERTISSEMENT:")
        print(f"  Seulement {len(solutions_uniques)} solution(s) unique(s) trouvée(s).")
        print(f"  Pour un robot 6-DOF typique, jusqu'à 8 solutions peuvent exister.")
        print(f"\n  Suggestions pour trouver plus de solutions:")
        print(f"    • Augmenter num_seeds (ex: 50, 100, 200)")
        print(f"    • Utiliser une méthode IK analytique si disponible")
        print(f"    • Vérifier la géométrie du robot (poignet sphérique?)")

    print()


def main():
    parser = argparse.ArgumentParser(
        description="Analyse l'unicité des solutions IK générées par CuRobo"
    )
    parser.add_argument(
        "--num_seeds",
        type=int,
        default=20,
        help="Nombre de graines IK (défaut: 20)",
    )
    parser.add_argument(
        "--seuil",
        type=float,
        default=0.01,
        help="Seuil d'unicité en radians (défaut: 0.01 rad ≈ 0.57°)",
    )
    parser.add_argument(
        "--test",
        action="store_true",
        help="Mode test avec solutions simulées",
    )

    args = parser.parse_args()

    # Créer l'analyseur
    analyzer = IKSolutionAnalyzer(seuil_unicite=args.seuil)

    # Générer ou charger les solutions
    if args.test:
        print(f"\n🧪 Mode TEST - Génération de solutions simulées...")
        solutions, success_flags = generer_solutions_test(args.num_seeds)
    else:
        print(f"\n❌ ERREUR: Mode production non encore implémenté.")
        print(f"   Utilisez --test pour tester avec des données simulées.")
        print(f"\n   Pour analyser de vraies solutions CuRobo:")
        print(f"     1. Intégrer ce code dans le service ROS /curobo_ik")
        print(f"     2. Ou créer un nœud ROS qui appelle le service et analyse les résultats")
        sys.exit(1)

    # Analyser l'unicité
    solutions_uniques, compteur = analyzer.filtrer_solutions_uniques(
        solutions, success_flags
    )

    # Calculer métriques de diversité
    metriques = analyzer.analyser_diversite(solutions_uniques)

    # Afficher le rapport
    afficher_rapport(
        args.num_seeds,
        solutions,
        success_flags,
        solutions_uniques,
        compteur,
        metriques,
        args.seuil,
    )


if __name__ == "__main__":
    main()
