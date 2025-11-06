from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def get_urdf_path_from_config(config_file_path, default_urdf_path):
    """
    Charge le paramètre urdf_path depuis le fichier YAML de configuration.
    Si le fichier ou le paramètre n'existe pas, retourne le chemin par défaut.
    """
    try:
        with open(config_file_path, 'r') as file:
            config = yaml.safe_load(file)
            urdf_path = config.get('robot_cfg', {}).get('kinematics', {}).get('urdf_path')
            if urdf_path:
                return urdf_path
    except (FileNotFoundError, yaml.YAMLError, KeyError, AttributeError) as e:
        print(f"Warning: Could not load urdf_path from config file: {e}")
        print(f"Using default urdf_path: {default_urdf_path}")

    return default_urdf_path


def launch_setup(context, *args, **kwargs):
    """
    Fonction appelée lors de l'exécution du launch pour résoudre les LaunchConfiguration
    """
    # Récupérer les valeurs résolues des arguments
    robot_config_file = LaunchConfiguration('robot_config_file').perform(context)
    urdf_path_arg = LaunchConfiguration('urdf_path').perform(context)

    # Chemin par défaut pour urdf_path
    default_urdf_path = os.path.join(
        get_package_share_directory('curobo_ros'),
        'curobo_doosan/src/m1013/',
        'm1013.urdf'
    )

    # Si urdf_path n'est pas fourni (égal au défaut), charger depuis robot_config_file
    # On vérifie si urdf_path est vide ou correspond au placeholder
    if not urdf_path_arg or urdf_path_arg == '':
        urdf_path_resolved = get_urdf_path_from_config(robot_config_file, default_urdf_path)
        print(f"[gen_traj.launch] Loading urdf_path from config: {urdf_path_resolved}")
    else:
        urdf_path_resolved = urdf_path_arg
        print(f"[gen_traj.launch] Using provided urdf_path: {urdf_path_resolved}")

    # Lire le contenu URDF depuis le fichier
    try:
        with open(urdf_path_resolved, 'r') as urdf_file:
            urdf_content = urdf_file.read()
    except FileNotFoundError:
        print(f"[gen_traj.launch] ERROR: URDF file not found: {urdf_path_resolved}")
        urdf_content = ""

    curobo_ros_launch_dir = os.path.join(
        get_package_share_directory('curobo_ros'), 'launch')

    include_realsense_launch = LaunchConfiguration('include_realsense_launch', default='false')

    from launch_ros.actions import Node as RosNode
    from launch.actions import DeclareLaunchArgument

    gui_enabled = LaunchConfiguration('gui', default='true')

    nodes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(curobo_ros_launch_dir, 'realsense.launch.py')),
            condition=IfCondition(include_realsense_launch)
        ),

        # Lancer les nœuds directement au lieu d'inclure launch_rviz2.launch.py
        # afin d'utiliser le bon URDF
        RosNode(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_content}]
        ),

        RosNode(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_gui': gui_enabled}]
        ),

        # Run curobo_gen_traj node
        Node(
            package='curobo_ros',
            executable='curobo_gen_traj',
            output='screen',
            parameters=[{
                'robot_config_file': LaunchConfiguration('robot_config_file')
            }]
        ),

        # Nodes pour la prévisualisation des trajectoires (remplace le fichier XML problématique)
        RosNode(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            namespace='preview',
            parameters=[{
                'source_list': ['/trajectory/joint_states']
            }]
        ),

        RosNode(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='preview',
            parameters=[{
                'robot_description': urdf_content,
                'frame_prefix': 'preview/'
            }]
        ),

        RosNode(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace='preview',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'preview/world']
        ),

        # Log an informational message
        LogInfo(
            msg='All nodes and launch files are launched'
        ),
    ]

    # Ajouter l'inclusion conditionnelle de RViz
    nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(curobo_ros_launch_dir, 'rviz_visualization.launch.py')
            ),
            condition=IfCondition(LaunchConfiguration('gui'))
        )
    )

    return nodes


def generate_launch_description():
    # Déclaration du répertoire de lancement de curobo_ros
    curobo_ros_launch_dir = os.path.join(
        get_package_share_directory('curobo_ros'), 'launch')

    # Chemin par défaut pour robot_config_file
    default_robot_config = os.path.join(
        get_package_share_directory('curobo_ros'),
        'curobo_doosan/src/m1013/',
        'm1013.yml'
    )

    # Déclaration de l'argument urdf_path (vide par défaut pour déclencher le chargement depuis YAML)
    declare_urdf_path = DeclareLaunchArgument(
        'urdf_path',
        default_value='',
        description='Chemin vers le fichier URDF du robot (si vide, chargé depuis robot_config_file)'
    )

    # Déclaration de l'argument robot_config_file
    declare_robot_config_file = DeclareLaunchArgument(
        'robot_config_file',
        default_value=default_robot_config,
        description='Chemin vers le fichier de configuration YAML du robot'
    )

    declare_include_realsense_launch = DeclareLaunchArgument(
        'include_realsense_launch',
        default_value='false',
        description='Inclure le fichier de lancement realsense.launch.py si défini à true'
    )

    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Lancer l\'interface graphique RViz (true/false)'
    )

    return LaunchDescription([
        # Définition des arguments de lancement
        declare_urdf_path,
        declare_robot_config_file,
        declare_include_realsense_launch,
        declare_gui,
        DeclareLaunchArgument(
            'max_attempts', default_value='2', description='Premier paramètre'
        ),
        DeclareLaunchArgument(
            'timeout', default_value='1', description='Deuxième paramètre (nombre)'
        ),
        DeclareLaunchArgument(
            'time_dilation_factor', default_value='0.5', description='Facteur de dilatation du temps'
        ),
        DeclareLaunchArgument(
            'voxel_size', default_value='1.0', description='Taille des voxels'
        ),
        DeclareLaunchArgument(
            'collision_activation_distance', default_value='0.5', description='Distance d\'activation de la collision'
        ),

        # Utiliser OpaqueFunction pour résoudre les LaunchConfiguration au moment de l'exécution
        OpaqueFunction(function=launch_setup)
    ])
