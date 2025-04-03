#!/usr/bin/env python3

"""
启动文件，用于启动麦克风静音控制节点
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成启动描述，启动麦克风静音控制节点"""
    
    # 声明是否默认静音的参数
    default_mute_arg = DeclareLaunchArgument(
        'default_mute',
        default_value='true',
        description='是否在启动时默认静音麦克风，等待主动发问触发后解除静音'
    )
    
    # 麦克风静音控制节点
    mic_mute_node = Node(
        package='robot_voice_launcher',
        executable='mic_mute_node',
        name='mic_mute_node',
        output='screen',
        parameters=[
            {'default_mute': LaunchConfiguration('default_mute')}
        ]
    )
    
    return LaunchDescription([
        default_mute_arg,
        mic_mute_node
    ])
