from setuptools import setup

package_name = 'arm_task_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/arm_task_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tzb',
    maintainer_email='tzb@example.com',
    description='arm task decision bridge',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_task_bridge = arm_task_bridge.node:main',
        ],
    },
)
