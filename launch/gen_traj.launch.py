from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import ast
import os
import yaml


def get_urdf_path_from_config(config_file_path, default_urdf_path):
    """
    Read urdf_path from the cuRobo robot config YAML.
    Falls back to default_urdf_path if the file or the key is missing.
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
    Read base_link from the cuRobo robot config YAML.
    Falls back to default_base_link if the file or the key is missing.
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
    Called at launch time, once LaunchConfiguration values can be resolved.
    """
    # Resolve the argument values
    robot_config_file = LaunchConfiguration('robot_config_file').perform(context)
    urdf_path_arg = LaunchConfiguration('urdf_path').perform(context)
    robot_name = LaunchConfiguration('robot').perform(context)

    # With a robot descriptor and no explicit robot_config_file override, derive
    # the cuRobo YAML (and base_link) from the descriptor.
    descriptor_base_link = None
    descriptor_joint_states_topic = None
    if robot_name:
        try:
            from curobo_ros.robot.robot_description import load_robot_description
            _desc = load_robot_description(robot_name)
            descriptor_base_link = _desc.base_link
            # Topic the robot publishes its real JointStates on, and the single
            # source of truth for it: the emulator uses /emulator/joint_states,
            # a real robot its own (M1013: /dsr01/joint_states).
            descriptor_joint_states_topic = _desc.strategy_params.get('joint_states_topic')
            if not robot_config_file:
                robot_config_file = _desc.curobo_config_path
            print(f"[gen_traj.launch] robot='{robot_name}' -> config {robot_config_file}")
        except Exception as e:
            print(f"[gen_traj.launch] Warning: could not load descriptor '{robot_name}': {e}")

    # Fallback urdf_path
    default_urdf_path = os.path.join(
        get_package_share_directory('curobo_ros'),
        'curobo_doosan/src/m1013/',
        'm1013.urdf'
    )

    # urdf_path defaults to the empty string: unset means "read it from
    # robot_config_file", an explicit value wins.
    if not urdf_path_arg or urdf_path_arg == '':
        urdf_path_resolved = get_urdf_path_from_config(robot_config_file, default_urdf_path)
        print(f"[gen_traj.launch] Loading urdf_path from config: {urdf_path_resolved}")
    else:
        urdf_path_resolved = urdf_path_arg
        print(f"[gen_traj.launch] Using provided urdf_path: {urdf_path_resolved}")

    # base_link: descriptor first, otherwise read from the cuRobo YAML.
    default_base_link = descriptor_base_link or "base_0"
    base_link = get_base_link_from_config(robot_config_file, default_base_link)
    print(f"[gen_traj.launch] Using base_link: {base_link}")

    # robot_state_publisher wants the URDF as a string, not a path.
    try:
        with open(urdf_path_resolved, 'r') as urdf_file:
            urdf_content = urdf_file.read()
    except FileNotFoundError:
        print(f"[gen_traj.launch] ERROR: URDF file not found: {urdf_path_resolved}")
        urdf_content = ""

    curobo_ros_launch_dir = os.path.join(
        get_package_share_directory('curobo_ros'), 'launch')

    from launch_ros.actions import Node as RosNode

    mapper_extent_xyz = ast.literal_eval(
        LaunchConfiguration('mapper_extent_xyz').perform(context))

    nodes = [
        # The state publishers are declared inline rather than pulled in from
        # another launch file, so they receive the URDF resolved above for the
        # selected robot instead of a hardcoded one.
        RosNode(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': urdf_content,
            }]
        ),

        RosNode(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                # Feedback topic of the selected robot, read from its descriptor.
                # This used to be hardcoded to the emulator, so RViz showed a
                # robot frozen at zero as soon as a real one was launched.
                'source_list': [
                    descriptor_joint_states_topic or '/emulator/joint_states'],
            }]
        ),

        # The planner itself (node name: unified_planner)
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
                # value_type is explicit on every forwarded scalar: without it
                # launch_ros infers the type from the string, so `voxel_size:=1`
                # would yield an int and fail the node's double declaration.
                'voxel_size': ParameterValue(
                    LaunchConfiguration('voxel_size'), value_type=float),
                # Planning retries per request (MotionPlanner.plan_pose).
                'max_attempts': ParameterValue(
                    LaunchConfiguration('max_attempts'), value_type=int),
                # Feedback publish period (s) during open-loop execution.
                'time_dilation_factor': ParameterValue(
                    LaunchConfiguration('time_dilation_factor'), value_type=float),
                # Build-time: baked into the MotionPlanner at construction.
                'collision_activation_distance': ParameterValue(
                    LaunchConfiguration('collision_activation_distance'),
                    value_type=float),
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

        # Trajectory preview pipeline (the translucent ghost robot in RViz).
        # /trajectory/joint_states is published by no node in this repository:
        # it comes from the trajectory_preview/TrajectoryPreviewPanel RViz panel
        # (rviz/rviz_curobo.rviz), which replays the /trajectory that
        # GhostStrategy publishes. It only looks orphaned -- do not "fix" it.
        RosNode(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            namespace='preview',
            parameters=[{
                'source_list': ['/trajectory/joint_states'],
            }]
        ),

        RosNode(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='preview',
            parameters=[{
                'robot_description': urdf_content,
                'frame_prefix': 'preview/',
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

    # RViz, included only when gui:=true
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
    # Empty by default, which triggers the lookup in robot_config_file.
    declare_urdf_path = DeclareLaunchArgument(
        'urdf_path',
        default_value='',
        description='Chemin vers le fichier URDF du robot (si vide, chargé depuis robot_config_file)'
    )
    # Robot selector (descriptor robots/<robot>.yaml). Drives the cuRobo YAML,
    # the base_link and the default control strategy.
    declare_robot = DeclareLaunchArgument(
        'robot',
        default_value='doosan_m1013',
        description='Nom du descripteur robot (robots/<robot>.yaml) ou chemin'
    )
    # Explicit override of the cuRobo YAML (empty = derived from the descriptor)
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
        declare_robot,
        declare_urdf_path,
        declare_robot_config_file,
        declare_camera_config_file,
        declare_gui,
        declare_world_file,
        # The defaults below MUST stay aligned with the declare_parameter() calls
        # in unified_planner_node.py: they are forwarded to the node and therefore
        # override its own defaults.
        DeclareLaunchArgument(
            'max_attempts', default_value='1',
            description='Planning retries per request (MotionPlanner.plan_pose)'
        ),
        DeclareLaunchArgument(
            'time_dilation_factor', default_value='0.5',
            description='Feedback publish period (s) during open-loop execution. '
                        'Not a speed control: set velocity/acceleration limits in the robot YAML cspace'
        ),
        DeclareLaunchArgument(
            'voxel_size', default_value='0.05',
            description='Voxel size (m) shared by the perception ESDF, the collision cache '
                        'and the published voxel grid. Smaller = finer, cubic VRAM cost'
        ),
        DeclareLaunchArgument(
            'mapper_extent_xyz', default_value='[2.56, 2.56, 2.56]',
            description="Étendue (m) de la grille voxel du Mapper, en repr Python d'une liste [x, y, z]"
        ),
        DeclareLaunchArgument(
            'collision_activation_distance', default_value='0.025',
            description='Distance (m) at which the collision cost activates. Build-time: '
                        'baked into the MotionPlanner, changing it at runtime needs a rebuild'
        ),

        # OpaqueFunction defers the body until LaunchConfigurations can be resolved
        OpaqueFunction(function=launch_setup)
    ])
