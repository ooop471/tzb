from setuptools import find_packages
from setuptools import setup

setup(
    name='rover_explore_launch',
    version='0.1.0',
    packages=find_packages(
        include=('rover_explore_launch', 'rover_explore_launch.*')),
)
