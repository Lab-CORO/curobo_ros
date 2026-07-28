"""
Launch de test pour les suites générées par ros2_test_compose.

Enveloppe mince autour de gen_traj.launch.py : mêmes nœuds, mêmes chemins de
résolution URDF/descripteur, seuls quatre arguments diffèrent de la
production pour rendre les suites rapides, déterministes et sans matériel :

- gui:=false             supprime RViz (évite ~15s de SIGINT->SIGTERM->SIGKILL
                          par suite et les codes de sortie -9/-6 qui polluent
                          assertExitCodes)
- voxel_size:=0.05        avec mapper_extent_xyz=[2.0, 2.0, 2.0] ci-dessous,
                          donne une grille 40^3 = 64000 voxels, ce que
                          test_object.yaml (cas 04) attend déjà
- mapper_extent_xyz       voir ci-dessus (2.0 / 0.05 = 40)
- robot:=emulator         aucun robot Doosan réel n'est présent en test
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    curobo_ros_launch_dir = os.path.join(
        get_package_share_directory('curobo_ros'), 'launch')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(curobo_ros_launch_dir, 'gen_traj.launch.py')
            ),
            launch_arguments={
                'gui': 'false',
                'voxel_size': '0.05',
                'mapper_extent_xyz': '[2.0, 2.0, 2.0]',
                'robot': 'emulator',
            }.items()
        ),
    ])
