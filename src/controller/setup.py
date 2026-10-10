from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ajaybhati',
    maintainer_email='ajaybhati@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'chaser_calculator = controller.chase_calculator_2:main',
            'kill_pose = controller.kill_pose_2:main',
            'evasion_calculator = controller.evasion_calculator_2:main',
            'goal_sender = controller.goal_sender_2:main',
            'state_machine = controller.state_machine_2:main',
        ],
    },
)
