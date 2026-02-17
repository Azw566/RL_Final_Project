import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'maze_mission'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='telemaque',
    maintainer_email='t.buzzi-menudier@studenti.unina.it',
    description='Bridge and Mapping logic for drone maze exploration',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'nav2_action_bridge = maze_mission.nav2_action_bridge:main',
            'aerial_mapper = maze_mission.aerial_mapper:main',
            'mission_planner = maze_mission.mission_planner:main',
        ],
    },
)
