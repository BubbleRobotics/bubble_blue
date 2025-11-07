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
    ('share/path_planner/launch', ['launch/rrt_path_follower.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luis Blunschi',
    maintainer_email='lublu@hotmail.ch',
    description='RRT path follower',
    license='MIT',
    entry_points={
    'console_scripts': [
        'rrt_path_follower = path_planner.mockup_rrt_follower:main',
    ],
    },
)
