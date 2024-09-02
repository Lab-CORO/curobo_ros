from setuptools import find_packages, setup

package_name = 'curobo_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'curobo_int_mark = curobo_ros.interfaces.interactive_marker:main',
            # 'test_fk = curobo_ros.test_fk:main'
        ],
    },
)
