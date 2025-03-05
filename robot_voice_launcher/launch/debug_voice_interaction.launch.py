#!/usr/bin/env python3

"""
调试语音交互启动文件
用于调试基于dummy_level和active_questioning_service触发的语音交互
封装以下三个命令：
1. ros2 launch robot_voice_launcher voice_system_stepfun.launch.py
2. ros2 run dummy_level_publisher dummy_level_publisher
3. ros2 run robot_voice_launcher service_trigger_node
"""

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    DeclareLaunchArgument
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

def generate_launch_description():
    """生成启动描述，启动语音交互调试所需的所有节点"""
    
    # 声明服务触发延迟参数
    service_trigger_delay_arg = DeclareLaunchArgument(
        'service_trigger_delay',
        default_value='3.0',
        description='Delay in seconds before triggering the active questioning service'
    )
    
    # 声明电梯信息发布频率参数
    publish_frequency_arg = DeclareLaunchArgument(
        'publish_frequency',
        default_value='5.0',
        description='Frequency for publishing dummy level information (Hz)'
    )
    
    # 语音系统启动文件
    voice_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('robot_voice_launcher'), 'launch', 'voice_system_stepfun.launch.py'])
        ]),
        # 这里可以添加launch参数覆盖，例如:
        # launch_arguments={'parameter_name': 'new_value'}.items()
    )
    
    # 模拟电梯楼层信息发布节点
    dummy_level_publisher_node = Node(
        package='dummy_level_publisher',
        executable='dummy_level_publisher',
        name='dummy_level_publisher_node',
        parameters=[{
            'publish_frequency': LaunchConfiguration('publish_frequency')
        }],
        output='screen'
    )
    
    # 服务触发节点
    service_trigger_node = Node(
        package='robot_voice_launcher',
        executable='service_trigger_node',
        name='service_trigger_node',
        parameters=[{
            'service_name': '/active_questioning/trigger_question',
            'delay': LaunchConfiguration('service_trigger_delay')
        }],
        output='screen'
    )
    
    # 按顺序启动所有节点
    return LaunchDescription([
        # 参数
        service_trigger_delay_arg,
        publish_frequency_arg,
        
        # 首先启动语音系统
        voice_system_launch,
        
        # 等待6秒后启动模拟电梯楼层信息发布节点（确保语音系统完全启动）
        TimerAction(
            period=6.0,
            actions=[dummy_level_publisher_node]
        ),
        
        # 等待7秒后启动服务触发节点（确保主动发问服务已经可用）
        TimerAction(
            period=7.0,
            actions=[service_trigger_node]
        )
    ])
