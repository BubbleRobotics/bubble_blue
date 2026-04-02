# src/path_planner/setup.py
from setuptools import setup

package_name = 'path_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/path_planner']),
        ('share/path_planner', ['package.xml']),
        ('share/path_planner/launch', [
            'launch/path_planner.launch.py',
            'launch/follower.launch.py',
            'launch/follower_gated.launch.py',
            'launch/simple_planner.launch.py',
            'launch/snake_planner.launch.py',
            'launch/odometry_enu_ned.launch.py',
            'launch/odometry_ned_enu.launch.py',
            'launch/simple_accel_controller.launch.py',
            'launch/optimized_trajectory.launch.py',
            'launch/full_sim_stack.launch.py',
        ]),
    ],
    install_requires=[
        'setuptools',
        'pandas',
    ],
    zip_safe=True,
    maintainer='Luis Blunschi',
    maintainer_email='lublu@hotmail.ch',
    description='RRT path follower',
    license='MIT',
    entry_points={
        'console_scripts': [
            'path_planner = path_planner.path_planner:main',
            'follower_node = path_planner.follower_node:main',
            'wait_mavros_ready = path_planner.wait_mavros_ready:main',
            'simple_planner = path_planner.simple_planner:main',
            'snake_planner = path_planner.snake_planner:main',
            'odometry_enu_ned = path_planner.odometry_enu_ned:main',
            'odometry_ned_enu = path_planner.odometry_ned_enu:main',
            'simple_accel_controller = path_planner.simple_accel_controller:main',
            'optimal_trajectory = path_planner.optimized_trajectory:main',
        ],
    },
)