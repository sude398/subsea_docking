from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'my_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zeyss',
    maintainer_email='zeyss@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'aruco_node_cam = my_pkg.aruco_node_cam:main',
            'aruco_node_video = my_pkg.aruco_node_video:main',
            'aruco_sub_node = my_pkg.aruco_sub_node:main',
            'aruco_detector_node = my_pkg.aruco_detector_node:main',
            'auto_control_node = my_pkg.auto_control_node:main',
            'deneme_video = my_pkg.deneme_video:main',
        ],
    },
)
