from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'speech_generation_baidu'

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
    description='ROS2功能包，使用百度语音合成API将文本转换为语音，订阅LLM响应并输出合成的音频',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'speech_generation_baidu_node = speech_generation_baidu.speech_generation_baidu_node:main'
        ],
    },
)
