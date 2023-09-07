from setuptools import setup
import os
from glob import glob
package_name = 'ultrasonics'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (os.path.join('lib', package_name), glob(os.path.join(package_name,'DFRobot_URM13.py'))),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'ultrasonic_publisher = ultrasonics.ultrasonic_node_publisher:main',
            'ultrasonic_listener = ultrasonics.ultrasonic_node_ALL_listener:main'


        ],
    },
)
