from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'llm_bytedance'

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
    install_requires=['setuptools', 'requests'],
    zip_safe=True,
    maintainer='jay',
    maintainer_email='jerryzhang7@126.com',
    description='ROS2节点，用于与火山引擎大模型API交互',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'llm_bytedance_node = llm_bytedance.llm_bytedance_node:main',
        ],
    },
)
