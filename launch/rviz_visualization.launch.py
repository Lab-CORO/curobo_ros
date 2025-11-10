from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import OpaqueFunction
import tempfile
import yaml


def setup_rviz_config(context, *args, **kwargs):
    """
    Fonction pour modifier le fichier de configuration RViz avec le Fixed Frame approprié
    """
    # Récupérer les arguments
    rviz_config_path = LaunchConfiguration('rviz_config').perform(context)
    base_link = LaunchConfiguration('base_link').perform(context)

    # Lire le fichier de configuration RViz original
    try:
        with open(rviz_config_path, 'r') as f:
            rviz_config = yaml.safe_load(f)

        # Modifier le Fixed Frame dans la configuration
        if 'Visualization Manager' in rviz_config:
            if 'Global Options' not in rviz_config['Visualization Manager']:
                rviz_config['Visualization Manager']['Global Options'] = {}
            rviz_config['Visualization Manager']['Global Options']['Fixed Frame'] = base_link

            # Modifier également le Reference Frame des displays curobo_rviz/ArrowInteractionDisplay
            if 'Displays' in rviz_config['Visualization Manager']:
                for display in rviz_config['Visualization Manager']['Displays']:
                    if isinstance(display, dict):
                        # Mettre à jour le Reference Frame pour les ArrowInteractionDisplay
                        if display.get('Class') == 'curobo_rviz/ArrowInteractionDisplay':
                            if 'Reference Frame' in display:
                                old_frame = display['Reference Frame']
                                display['Reference Frame'] = base_link
                                print(f"[rviz_visualization.launch] Updated Reference Frame for '{display.get('Name', 'Unknown')}': {old_frame} -> {base_link}")

        # Créer un fichier temporaire avec la configuration modifiée
        temp_file = tempfile.NamedTemporaryFile(mode='w', suffix='.rviz', delete=False)
        yaml.dump(rviz_config, temp_file, default_flow_style=False)
        temp_config_path = temp_file.name
        temp_file.close()

        print(f"[rviz_visualization.launch] Using Fixed Frame: {base_link}")
        print(f"[rviz_visualization.launch] Generated temp config: {temp_config_path}")

    except Exception as e:
        print(f"[rviz_visualization.launch] Warning: Could not modify RViz config: {e}")
        print(f"[rviz_visualization.launch] Using original config: {rviz_config_path}")
        temp_config_path = rviz_config_path

    # Nœud RViz2 avec la configuration modifiée
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', temp_config_path],
        parameters=[{
            'base_link': base_link
        }]
    )

    return [rviz_node]


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

    # Déclaration de l'argument pour le base_link
    declare_base_link = DeclareLaunchArgument(
        'base_link',
        default_value='base_0',
        description='Frame ID racine du robot (utilisé comme Fixed Frame dans RViz)'
    )

    return LaunchDescription([
        declare_rviz_config,
        declare_base_link,
        OpaqueFunction(function=setup_rviz_config)
    ])
