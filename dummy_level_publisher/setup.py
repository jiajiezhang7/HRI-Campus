from setuptools import find_packages, setup

package_name = 'dummy_level_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='agilex03',
    maintainer_email='jerryzhang7@126.com',
    description='模拟电梯楼层信息发布节点',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dummy_level_publisher = dummy_level_publisher.dummy_level_publisher_node:main',
        ],
    },
)
