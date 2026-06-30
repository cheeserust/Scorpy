from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vicpinky_task_servers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='hari',
    maintainer_email='hari@example.com',
    description='VicPinky task action servers',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
	    'console_scripts': [
		'nav_go_to_server = vicpinky_task_servers.nav_go_to_server:main',
		'dock_align_server = vicpinky_task_servers.dock_align_server:main',
		'elevator_door_server = vicpinky_task_servers.elevator_door_server:main',
		'floor_check_server = vicpinky_task_servers.floor_check_server:main',
	    ],
	},
)
