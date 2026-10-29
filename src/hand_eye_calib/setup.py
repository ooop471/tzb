from glob import glob
from setuptools import setup

package_name = 'hand_eye_calib'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tzb',
    maintainer_email='tzb@example.com',
    description='Eye-in-hand camera calibration with live capture and offline solver.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_eye_calibration = hand_eye_calib.hand_eye_node:main',
            'offline_calibrate = hand_eye_calib.offline_calibrate:main',
        ],
    },
)
