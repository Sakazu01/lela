from setuptools import setup
import os
from glob import glob

package_name = 'real'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, package_name + '.nodes', package_name + '.utils'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lela_team',
    maintainer_email='lela@example.com',
    description='LELA autonomous payload dropping system',
    license='MIT',
    entry_points={
        'console_scripts': [
            'state_manager = real.nodes.state_manager:main',
            'mission_monitor = real.nodes.mission_monitor:main',
            'camera_controller = real.nodes.camera_controller:main',
            'color_detector = real.nodes.color_detector:main',
            'drop_calculator = real.nodes.drop_calculator:main',
            'servo_controller = real.nodes.servo_controller:main',
        ],
    },
)