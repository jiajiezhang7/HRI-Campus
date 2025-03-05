from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_voice_launcher'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jay',
    maintainer_email='jerryzhang7@126.com',
    description='统一启动包，用于顺序启动语音交互相关的所有节点',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'active_questioning_node = robot_voice_launcher.active_questioning_node:main',
            'mic_mute_node = robot_voice_launcher.mic_mute_node:main',
            'interaction_coordinator_node = robot_voice_launcher.interaction_coordinator_node:main',
            'service_trigger_node = robot_voice_launcher.service_trigger_node:main',
        ],
    },
)
