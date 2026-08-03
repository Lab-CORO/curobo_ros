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
    Rewrite the RViz config with the Fixed Frame of the selected robot.

    The shipped rviz_curobo.rviz hardcodes one Fixed Frame; a robot whose base
    link differs would open on an empty view. Rather than ship one config per
    robot, the file is patched into a temporary copy at launch time.
    """
    rviz_config_path = LaunchConfiguration('rviz_config').perform(context)
    base_link = LaunchConfiguration('base_link').perform(context)

    try:
        with open(rviz_config_path, 'r') as f:
            rviz_config = yaml.safe_load(f)

        if 'Visualization Manager' in rviz_config:
            if 'Global Options' not in rviz_config['Visualization Manager']:
                rviz_config['Visualization Manager']['Global Options'] = {}
            rviz_config['Visualization Manager']['Global Options']['Fixed Frame'] = base_link

        # delete=False: RViz opens the file after this function returns.
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

    # On failure the except branch above falls back to the original config, so
    # this always receives a readable path.
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
    Start RViz. Included by gen_traj.launch.py when gui:=true.
    """

    rviz_config = PathJoinSubstitution([
        FindPackageShare('curobo_ros'),
        'rviz/rviz_curobo.rviz'
    ])

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config,
        description='Chemin vers le fichier de configuration RViz'
    )

    # Passed down by gen_traj.launch.py, which resolves it from the descriptor.
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
