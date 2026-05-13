from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sim_robot_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    description='PyBullet simulation of xArm7 for CoM estimation',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'sim_world = sim_robot_pkg.main:main',
            'bottle_check = sim_robot_pkg.bottle_check:main',
            'pick_place_demo = sim_robot_pkg.pick_place_demo:main',
        ],
    },
)
