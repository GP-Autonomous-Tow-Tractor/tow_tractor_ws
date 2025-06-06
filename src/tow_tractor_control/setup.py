from setuptools import find_packages, setup

package_name = 'tow_tractor_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mahmoud',
    maintainer_email='mah2002moud@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rear_wheel_steering_controller = tow_tractor_control.rear_wheel_steering_controller_node:main',
            'diff_drive_controller = tow_tractor_control.diff_drive_controller_node:main',
        ],
    },
)
