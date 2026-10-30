from glob import glob
import os

from setuptools import setup


package_name = 'mineral_nav_goal'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tzb',
    maintainer_email='tzb@example.com',
    description='YOLO and aligned depth based mineral grasp stop goal',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'mineral_nav_goal = mineral_nav_goal.node:main',
        ],
    },
)
