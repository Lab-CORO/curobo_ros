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
import ast
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


def get_base_link_from_config(config_file_path, default_base_link):
    """
    Charge le paramètre base_link depuis le fichier YAML de configuration.
    Si le fichier ou le paramètre n'existe pas, retourne le frame par défaut.
    """
    try:
        with open(config_file_path, 'r') as file:
            config = yaml.safe_load(file)
            base_link = config.get('robot_cfg', {}).get('kinematics', {}).get('base_link')
            if base_link:
                return base_link
    except (FileNotFoundError, yaml.YAMLError, KeyError, AttributeError) as e:
        print(f"Warning: Could not load base_link from config file: {e}")
        print(f"Using default base_link: {default_base_link}")

    return default_base_link


def launch_setup(context, *args, **kwargs):
    """
    Fonction appelée lors de l'exécution du launch pour résoudre les LaunchConfiguration
    """
    # Récupérer les valeurs résolues des arguments
    robot_config_file = LaunchConfiguration('robot_config_file').perform(context)
    urdf_path_arg = LaunchConfiguration('urdf_path').perform(context)
    robot_name = LaunchConfiguration('robot').perform(context)

    # Si un robot (descripteur) est fourni et que robot_config_file n'a pas été
    # surchargé explicitement, dériver le YAML curobo (+ base_link) du descripteur.
    descriptor_base_link = None
    if robot_name:
        try:
            from curobo_ros.robot.robot_description import load_robot_description
            _desc = load_robot_description(robot_name)
            descriptor_base_link = _desc.base_link
            if not robot_config_file:
                robot_config_file = _desc.curobo_config_path
            print(f"[gen_traj.launch] robot='{robot_name}' -> config {robot_config_file}")
        except Exception as e:
            print(f"[gen_traj.launch] Warning: could not load descriptor '{robot_name}': {e}")

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

    # base_link : descripteur en priorité, sinon extrait du YAML curobo.
    default_base_link = descriptor_base_link or "base_0"
    base_link = get_base_link_from_config(robot_config_file, default_base_link)
    print(f"[gen_traj.launch] Using base_link: {base_link}")

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
    mapper_extent_xyz = ast.literal_eval(
        LaunchConfiguration('mapper_extent_xyz').perform(context))

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
            parameters=[{
                'robot_description': urdf_content,
                'base_link': base_link
            }]
        ),

        RosNode(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_gui': gui_enabled,
                'source_list': ['/emulator/joint_states'],
                'base_link': base_link
            }]
        ),

        # Run curobo_gen_traj node
        Node(
            package='curobo_ros',
            executable='curobo_trajectory_planner',
            output='screen',
            parameters=[{
                'robot': robot_name,
                'robot_config_file': LaunchConfiguration('robot_config_file'),
                'cameras_config_file': LaunchConfiguration('cameras_config_file'),
                'base_link': base_link,
                'world_file': LaunchConfiguration('world_file'),
                # ESDF/voxel resolution (shared by Mapper ESDF, collision cache
                # and the published voxel grid). Forwarded from leeloo.
                'voxel_size': LaunchConfiguration('voxel_size'),
                # Output voxel grid geometry for the U-Net consumer:
                # 128^3 @ 0.02 m centred on the robot base (extent = 128 * 0.02).
                'mapper_extent_xyz': mapper_extent_xyz,
                'mapper_grid_center': [0.0, 0.0, 0.0],
                # Mapper depth-projection resolution. The Mapper allocates its
                # GPU projection kernel ONCE for this exact (H, W), so every
                # depth frame fed to mapper.integrate() must be resized to this
                # resolution (and its intrinsics scaled accordingly) before
                # integration — see camera_depth_map_strategy.callback_depth_map.
                # Defaults to the Azure Kinect depth_to_rgb size (1280x720).
                # Controllable as ROS params: mapper_image_width / mapper_image_height.
                'mapper_image_width': 1280,
                'mapper_image_height': 720,
                # Sparse voxel topic publish rate (Hz); <= 0 disables it.
                'sparse_voxel_publish_rate': 7.0,
            }]
        ),

        # Nodes pour la prévisualisation des trajectoires (remplace le fichier XML problématique)
        RosNode(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            namespace='preview',
            parameters=[{
                'source_list': ['/trajectory/joint_states'],
                'base_link': base_link
            }]
        ),

        RosNode(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='preview',
            parameters=[{
                'robot_description': urdf_content,
                'frame_prefix': 'preview/',
                'base_link': base_link
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
            launch_arguments={'base_link': base_link}.items(),
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
    # Sélecteur de robot (descripteur robots/<robot>.yaml). Pilote le YAML curobo,
    # le base_link et la stratégie par défaut.
    declare_robot = DeclareLaunchArgument(
        'robot',
        default_value='doosan_m1013',
        description='Nom du descripteur robot (robots/<robot>.yaml) ou chemin'
    )
    # robot_config_file : override explicite du YAML curobo (vide = dérivé du descripteur)
    declare_robot_config_file = DeclareLaunchArgument(
        'robot_config_file',
        default_value='',
        description='Override du YAML curobo (si vide, dérivé du descripteur robot)'
    )
    declare_camera_config_file = DeclareLaunchArgument(
        'cameras_config_file',
        default_value='',
        description='Chemin vers le fichier de configuration YAML des cameras'
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

    declare_world_file = DeclareLaunchArgument(
        'world_file',
        default_value='',
        description='Chemin vers le fichier de configuration du monde (world config YAML)'
    )

    return LaunchDescription([
        # Définition des arguments de lancement
        declare_robot,
        declare_urdf_path,
        declare_robot_config_file,
        declare_camera_config_file,
        declare_include_realsense_launch,
        declare_gui,
        declare_world_file,
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
            'mapper_extent_xyz', default_value='[2.56, 2.56, 2.56]',
            description="Étendue (m) de la grille voxel du Mapper, en repr Python d'une liste [x, y, z]"
        ),
        DeclareLaunchArgument(
            'collision_activation_distance', default_value='0.5', description='Distance d\'activation de la collision'
        ),

        # Utiliser OpaqueFunction pour résoudre les LaunchConfiguration au moment de l'exécution
        OpaqueFunction(function=launch_setup)
    ])
