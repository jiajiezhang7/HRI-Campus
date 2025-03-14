from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'audio_play_python'

setup(
    name=package_name,
    version='0.0.0',
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
    description='ROS2节点，用于将音频数据播放到系统扬声器',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'audio_play_node = audio_play_python.audio_play_node:main'
        ],
    },
)
