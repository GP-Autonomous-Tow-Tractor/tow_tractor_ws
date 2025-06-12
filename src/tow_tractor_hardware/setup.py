from setuptools import setup
from glob import glob
import os

package_name = 'tow_tractor_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name,
        'hardware',
        'hardware.tests',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),  glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='tow_tractor Hardware ROS2 Package',
    license='Your License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_receiver_node = tow_tractor_hardware.sensor_receiver_node:main',
            'actuators_sender_node = tow_tractor_hardware.actuators_sender_node:main',
            'odometry_publisher_node = tow_tractor_hardware.odometry_publisher_node:main',
        ],
    },
)
