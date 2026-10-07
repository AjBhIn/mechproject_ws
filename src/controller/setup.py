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
        ('lib/' + package_name, glob('scripts/*.py')),

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
            'chaser = controller.chaser_controller:main',
            'target_broadcaster = controller.target_broadcaster:main',
            'evasion_calculator = controller.evasion_calculator:main',
            'escape_goal_sender = controller.escape_goal_sender:main',
        ],
    },
)
