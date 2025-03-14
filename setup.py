from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'drone_autonav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.*')),
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
            'drone_autonav_node = drone_autonav.drone_autonav_node:main',
            'send_goal = drone_autonav.goal:main',
        ],
    },
)
