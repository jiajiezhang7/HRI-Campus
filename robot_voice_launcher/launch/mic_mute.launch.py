#!/usr/bin/env python3

"""
启动文件，用于启动麦克风静音控制节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """生成启动描述，启动麦克风静音控制节点"""
    
    # 麦克风静音控制节点
    mic_mute_node = Node(
        package='robot_voice_launcher',
        executable='mic_mute_node',
        name='mic_mute_node',
        output='screen'
    )
    
    return LaunchDescription([
        mic_mute_node
    ])
