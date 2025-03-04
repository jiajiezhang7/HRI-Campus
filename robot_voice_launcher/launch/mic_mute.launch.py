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
    
    # 声明静音持续时间参数
    mute_duration_arg = DeclareLaunchArgument(
        'mute_duration',
        default_value='5.0',
        description='Duration to keep microphone muted after audio playback (seconds)'
    )
    
    # 麦克风静音控制节点
    mic_mute_node = Node(
        package='robot_voice_launcher',
        executable='mic_mute_node',
        name='mic_mute_node',
        parameters=[{
            'mute_duration': LaunchConfiguration('mute_duration'),
        }],
        output='screen'
    )
    
    return LaunchDescription([
        mute_duration_arg,
        mic_mute_node
    ])
