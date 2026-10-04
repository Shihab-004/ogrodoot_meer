import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_vision'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shihab',
    maintainer_email='shihabul.islam.shihab.004@gmail.com',
    description='Computer Vision Subsystem for Tree Vitality Detection — MEER Rover NAGC 2026',
    license='MIT',
    entry_points={
        'console_scripts': [
            'tree_vitality_node = rover_vision.tree_vitality_node:main',
        ],
    },
)
