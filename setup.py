from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'curobo_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),                                      glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'curobo_doosan/src/m1013'),                           glob(os.path.join('curobo_doosan/src/m1013', '*.*'))),
        (os.path.join('share', package_name, 'curobo_doosan/src/m1013/meshes', 'm1013_white'),     glob(os.path.join('curobo_doosan/src/m1013/meshes/m1013_white', '*.dae*'))),
        (os.path.join('share', package_name, 'curobo_doosan/src/m1013/meshes', 'm1013_collision'), glob(os.path.join('curobo_doosan/src/m1013/meshes/m1013_collision', '*.dae*'))), #TODO faire une packages ros2 pour la config doosan
        (os.path.join('share', package_name, 'rviz'),                                        glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'),                                        glob('config/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='will',
    maintainer_email='will@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = curobo_ros.publisher_member_function:main',
            'listener = curobo_ros.subscriber_member_function:main',
            'curobo_ik = curobo_ros.core.ik:main',
            'curobo_fk = curobo_ros.core.fk:main',
            'curobo_gen_traj = curobo_ros.core.generate_trajectory:main',
            'curobo_mpc = curobo_ros.core.mpc:main',
            'curobo_trajectory_planner = curobo_ros.core.unified_planner_node:main',
            'curobo_int_mark = curobo_ros.interfaces.simple_arrow:main',
            'viz_voxel_grid = curobo_ros.interfaces.show_voxel_grid:main',
            'robot_segmentation = curobo_ros.core.robot_segmentation:main',
            'test_pointcloud_camera = curobo_ros.core.test_pointcloud_camera:main',
        ],
    },
)
