from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'curobo_ros'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),                                      glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'robots'),                                      glob('robots/*.yaml')),
        (os.path.join('share', package_name, 'curobo_doosan/src/m1013'),                           glob(os.path.join('curobo_doosan/src/m1013', '*.*'))),
        (os.path.join('share', package_name, 'curobo_doosan/src/m1013/meshes', 'm1013_white'),     glob(os.path.join('curobo_doosan/src/m1013/meshes/m1013_white', '*.dae*'))),
        (os.path.join('share', package_name, 'curobo_doosan/src/m1013/meshes', 'm1013_collision'), glob(os.path.join('curobo_doosan/src/m1013/meshes/m1013_collision', '*.dae*'))),
        (os.path.join('share', package_name, 'curobo_doosan/src/m1013/meshes', 'm1013_blue'),      glob(os.path.join('curobo_doosan/src/m1013/meshes/m1013_blue', '*.dae*'))), #TODO faire une packages ros2 pour la config doosan
        (os.path.join('share', package_name, 'rviz'),                                        glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'),                                        glob('config/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Guillaume Dupoiron',
    maintainer_email='guillaume.dupoiron@protonmail.com',
    description='GPU-accelerated motion planning for ROS 2, powered by cuRobo.',
    license='Apache-2.0',
    # setuptools 81 dropped `tests_require`; it now warns "Unknown distribution
    # option" and the key never reaches colcon. colcon's has_test_dependency()
    # then reports no pytest dependency, PytestPythonTestingStep.match() fails,
    # and `colcon test` silently falls back to `python -m unittest` -> "NO TESTS
    # RAN". extras_require is the supported spelling and colcon reads it.
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'curobo_trajectory_planner = curobo_ros.core.unified_planner_node:main',
            'robot_segmentation = curobo_ros.core.robot_segmentation:main',
            # Tools. `ros2 run` only looks in lib/<pkg>/, where console_scripts
            # are the only thing that lands: a file installed through data_files
            # goes to share/ and stays invisible to `ros2 run`.
            'benchmark_voxel_grid = curobo_ros.tools.benchmark_voxel_grid:main',
            'run_pose_matrix = curobo_ros.tools.run_pose_matrix:main',
        ],
    },
)
