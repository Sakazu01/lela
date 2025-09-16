from setuptools import setup
import os
from glob import glob

package_name = 'opencv_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),  # Include all Python launch files
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'numpy',
    ],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Computer vision package for color detection using OpenCV',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'deteksi_warna = opencv_pkg.deteksi_warna:main',
            'program_warna = opencv_pkg.program_warna:main',
            'hsv_config = opencv_pkg.hsv_config:main',
            'dropping = opencv_pkg.dropping:main',
            'dummy_vfr = opencv_pkg.dummy_vfr:main',
            'servo_controller = opencv_pkg.servo_controller:main',
            'status_text_sender = opencv_pkg.status_text_sender:main',
        ],
    },
)