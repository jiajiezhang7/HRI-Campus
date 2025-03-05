from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'speech_recognition_stepfun'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools', 'numpy', 'requests'],
    zip_safe=True,
    maintainer='jay',
    maintainer_email='jerryzhang7@126.com',
    description='ROS2 package for speech recognition using StepFun API',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'speech_recognition_stepfun_node = speech_recognition_stepfun.speech_recognition_stepfun_node:main'
        ],
    },
)
