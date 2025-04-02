from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_animation_display'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 安装网页和图片资源文件
        (os.path.join('share', package_name, 'web'), glob('resource/web/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='agilex03',
    maintainer_email='jerryzhang7@126.com',
    description='用于机器人说话时在第二屏幕上显示动画的功能包',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tts_status_publisher = robot_animation_display.tts_status_publisher:main',
            'mock_tts_status = robot_animation_display.mock_tts_status:main'
        ],
    },
)
