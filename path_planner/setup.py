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
            'launch/snake_planner.launch.py',
            'launch/odometry_ned_enu.launch.py',
            'launch/optimized_trajectory.launch.py',
            'launch/full_sim_stack.launch.py',
            'launch/ego_obstacle_evaluation.launch.py',
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
            'snake_planner = path_planner.snake_planner:main',
            'odometry_ned_enu = path_planner.odometry_ned_enu:main',
            'optimal_trajectory = path_planner.optimized_trajectory:main',
            'run_test_MT = path_planner.run_test_MT:main',
            'ego_obstacle_evaluation = path_planner.ego_obstacle_evaluation:main',        
        ],
    },
)