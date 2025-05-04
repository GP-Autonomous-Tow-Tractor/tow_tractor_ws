from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'tow_tractor_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'config'),  glob('config/*')),
        (os.path.join('share', package_name, 'launch'),  glob('launch/*')),
        (os.path.join('share', package_name, 'meshes'),  glob('meshes/*')),
        (os.path.join('share', package_name, 'urdf', 'stupid_robot'),  glob('urdf/stupid_robot/*')),
        (os.path.join('share', package_name, 'urdf', 'tow_tractor_v1'),  glob('urdf/tow_tractor_v1/*')),
        (os.path.join('share', package_name, 'urdf', 'tow_tractor_v2'),  glob('urdf/tow_tractor_v2/*')),
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
            'model_info_publisher = tow_tractor_description.model_info_publisher_node:main',
        ],
    },
)
