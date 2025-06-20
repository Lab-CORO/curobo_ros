from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Déclaration du répertoire de lancement de curobo_ros
    curobo_ros_launch_dir = os.path.join(
        get_package_share_directory('curobo_ros'), 'launch')

    urdf_file_name = 'm1013.urdf'

    urdf = Command(['cat ', PathJoinSubstitution(
        [FindPackageShare('curobo_ros'), 'curobo_doosan/src/m1013/', urdf_file_name])])
   
    declare_include_realsense_launch = DeclareLaunchArgument(
        'include_realsense_launch',
        default_value='false',
        description='Inclure le fichier de lancement realsense.launch.py si défini à true'
    )



    include_realsense_launch = LaunchConfiguration('include_realsense_launch', default = 'false')

    return LaunchDescription([
        # Définition des arguments de lancement
        DeclareLaunchArgument(
            'max_attempts', default_value='2', description='Premier paramètre'
        ),
        DeclareLaunchArgument(
            'timeout', default_value='1', description='Deuxième paramètre (nombre)'
        ),
        DeclareLaunchArgument(
            'time_dilation_factor', default_value='0.01', description='Facteur de dilatation du temps'
        ),
        DeclareLaunchArgument(
            'voxel_size', default_value='1.0', description='Taille des voxels'
        ),
        DeclareLaunchArgument(
            'collision_activation_distance', default_value='0.5', description='Distance d\'activation de la collision'
        ),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(curobo_ros_launch_dir, 'realsense.launch.py')),
            condition=IfCondition(include_realsense_launch)
        ),
       
        # Include the RViz2 launch file from curobo_ros
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(curobo_ros_launch_dir, 'launch_rviz2.launch.py')
            ),
            launch_arguments={
                'max_attempts': LaunchConfiguration('max_attempts'),
                'timeout': LaunchConfiguration('timeout'),
                'time_dilation_factor': LaunchConfiguration('time_dilation_factor'),
                'voxel_size': LaunchConfiguration('voxel_size'),
                'collision_activation_distance':LaunchConfiguration('collision_activation_distance')
            }.items()
        ),

        # Run curobo_fk node
        Node(
            package='curobo_ros',
            executable='curobo_fk',
            name='curobo_fk',
            output='screen'
        ),

        # Run curobo_gen_traj node
        Node(
            package='curobo_ros',
            executable='curobo_gen_traj',
            output='screen'
        ),

        # Run curobo_int_mark node
        # Node(
        #     package='curobo_ros',
        #     executable='curobo_int_mark',
        #     name='curobo_int_mark',
        #     output='screen'
        # ),

        # Include the trajectory_preview launch file
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(curobo_ros_launch_dir,
                             'robot_model_preview_pipeline.launch.xml')
            ),
            launch_arguments={
                'robot_description': urdf,
                'root_frame': 'world',
            }.items(),
        ),

        # Log an informational message
        LogInfo(
            msg='All nodes and launch files are launched'
        ),
    ])
