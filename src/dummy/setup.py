import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'dummy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='banger',
    maintainer_email='banger@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'opencv_node = dummy.opencv_node:main',
        'dropping_node = dummy.dropping_node:main',
        'vfr_simulator = dummy.vfr_simulator:main',
        'servo_node = dummy.servo_node:main',
        'waypoint_monitor = dummy.waypoint_monitor:main',
    ],
},
)
