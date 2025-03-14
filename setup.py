from setuptools import find_packages, setup

package_name = 'drone_autonav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/drone_autonav.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='intel',
    maintainer_email='01fe21bee114@kletech.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    extras_require={'test':['pytest'],},
    entry_points={
        'console_scripts': [
            'depth_processor = drone_autonav.depth_processor:main',
            'occupancy_mapper = drone_autonav.occupancy_mapper:main',
            'path_planner = drone_autonav.path_planner:main',
            'trajectory_controller = drone_autonav.trajectory_controller:main',
            'drone_autonav_node = drone_autonav.drone_autonav_node:main',
        ],
    },
)
