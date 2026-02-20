from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'maze_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maze Team',
    maintainer_email='user@example.com',
    description='Core mission logic for leader-follower maze exploration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav2_action_bridge = maze_mission.nav2_action_bridge:main',
            'mission_planner = maze_mission.mission_planner:main',
            'aruco_detector = maze_mission.aruco_detector:main',
            'origin_sync_node = maze_mission.origin_sync_node:main',
            'turtlebot_controller = maze_mission.turtlebot_controller:main',
        ],
    },
)
