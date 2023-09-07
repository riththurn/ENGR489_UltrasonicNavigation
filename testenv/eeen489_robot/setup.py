from setuptools import setup
import os
from glob import glob
package_name = 'eeen489_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('lib', package_name), glob(os.path.join(package_name,'roboclaw_3.py'))),
        (os.path.join('lib', package_name), glob(os.path.join(package_name,'motorClass.py'))),
        (os.path.join('share', package_name), glob(os.path.join('launch','*.launch.py'))),
        (os.path.join('share', package_name), glob(os.path.join('config','*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
    'low_level_ros_interface_node = eeen489_robot.low_level_ros_interface_node:main',
    'high_level_ros_interface_node = eeen489_robot.high_level_ros_interface_node:main'
        ],
    },
)
