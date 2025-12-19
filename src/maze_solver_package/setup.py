import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'maze_solver_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'videos'), glob('videos/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='junaed',
    maintainer_email='junaed@umn.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simple_drive = maze_solver_package.simple_drive:main',
            'wall_follower = maze_solver_package.wall_follower:main',
        ],
    },
)
