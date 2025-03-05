from setuptools import setup
import os
from glob import glob

package_name = 'speech_generation_stepfun'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'requests'],
    zip_safe=True,
    maintainer='jay',
    maintainer_email='jerryzhang7@126.com',
    description='ROS2 package for speech generation using StepFun API',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'speech_generation_stepfun_node = speech_generation_stepfun.speech_generation_stepfun_node:main'
        ],
    },
)
