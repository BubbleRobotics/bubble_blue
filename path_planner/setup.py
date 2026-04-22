# src/path_planner/setup.py
from setuptools import setup
from glob import glob

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
            'launch/seabed_scan.launch.py',
            'launch/seabed_scan_forward_only.launch.py',
            'launch/follower_live_test.launch.py',
            'launch/odometry_enu_ned.launch.py',
            'launch/odometry_enu_ned_sim.launch.py',
            'launch/simple_accel_controller.launch.py',
            'launch/run_test_MT.launch.py',
            'launch/snake_planner.launch.py',
        ]),
        ('share/path_planner/config', [
            'config/seabed_scan.yaml',
            'config/seabed_scan_forward_only.yaml',
        ]),
        ('share/path_planner/optimized_trajectories',
            glob('optimized_trajectories/*.csv')
        ),
    ],
    install_requires=['setuptools'],
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
            'seabed_scan_planner = path_planner.seabed_scan_planner:main',
            'seabed_scan_planner_zig_zag = path_planner.seabed_scan_planner_zig_zag:main',
            'seabed_scan_planner_forward_only = path_planner.seabed_scan_planner_forward_only:main',
            'follower_live_test = path_planner.follower_node_live_test:main',
            'odometry_enu_ned = path_planner.odometry_enu_ned:main',
            'odometry_enu_ned_sim = path_planner.odometry_enu_ned_sim:main',
            'simple_accel_controller = path_planner.simple_accel_controller:main',
            'odometry_ned_enu = path_planner.odometry_ned_enu:main',
            'run_test_MT = path_planner.run_test_MT:main',
            'snake_planner = path_planner.snake_planner:main',
        ],
    },
)