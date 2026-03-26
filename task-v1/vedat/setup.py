from setuptools import find_packages, setup

package_name = 'aruco_docking'

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
    maintainer='vedat',
    maintainer_email='gulv22@itu.edu.tr',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'docking_node = aruco_docking.docking_mission_node:main',
            'mission_listener = aruco_docking.mission_listener_node:main',
        ],
    },
)
