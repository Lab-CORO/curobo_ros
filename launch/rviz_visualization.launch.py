from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Fichier de lancement pour la visualisation RViz.
    Ce fichier lance RViz avec la configuration appropriée.
    """

    # Configuration RViz par défaut
    rviz_config = PathJoinSubstitution([
        FindPackageShare('curobo_ros'),
        'rviz/rviz_curobo.rviz'
    ])

    # Déclaration de l'argument pour le fichier de configuration RViz
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config,
        description='Chemin vers le fichier de configuration RViz'
    )

    # Nœud RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )

    return LaunchDescription([
        declare_rviz_config,
        rviz_node
    ])
