from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'niryo_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rahmlab',
    maintainer_email='rahmlab@todo.todo',
    description='Niryo Ned2 teleoperation: master/actor nodes + teleop controller.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'master_node = niryo_teleop.master.master_node:main',
            'master_joint_state_publisher = niryo_teleop.master.master_joint_state_publisher:main',
            'teleop_controller_node = niryo_teleop.teleop.teleop_controller_node:main',
            'teleoperation_controller = niryo_teleop.teleop.teleop_controller_node:main',
            'actor_controller = niryo_teleop.actor.actor_controller:main',
            'actor_joint_state_publisher = niryo_teleop.actor.actor_joint_state_publisher:main',
        ],
    },
)
