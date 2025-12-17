from setuptools import setup
from glob import glob
import os

package_name = 'maze_bringup_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Marker file for ament index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # package.xml install
        ('share/' + package_name, ['package.xml']),
        # Launch / config / worlds
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        ('share/' + package_name + '/launch', [
            'launch/single_robot_launch.py',
        ]),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Vu',
    maintainer_email='vu000194@umn.edu',
    description='Bringup and launch files for the multi-robot maze project.',
    license='Apache-2.0',
)
