from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'elevator_interaction_flexbe_behaviors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/ament_index/resource_index/flexbe_behaviors',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/elevator_interaction.launch.py']),
        # Add manifest files
        (os.path.join('lib', package_name, 'manifest'),
            glob('manifest/*.xml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='agilex03',
    maintainer_email='jerryzhang7@126.com',
    description='FlexBE behaviors for elevator interaction system',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'elevator_interaction_behavior = elevator_interaction_flexbe_behaviors.elevator_interaction_behavior_sm:main',
        ],
    },
)
