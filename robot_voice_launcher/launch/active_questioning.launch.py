#!/usr/bin/env python3

"""
主动发问启动文件
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """生成启动描述，启动主动发问节点"""
    
    # 声明延迟时间参数
    delay_arg = DeclareLaunchArgument(
        'delay',
        default_value='1.0',  # 默认延迟1秒
        description='Delay in seconds before asking the question after service call'
    )
    
    # 主动发问节点
    active_questioning_node = Node(
        package='robot_voice_launcher',
        executable='active_questioning_node',
        name='active_questioning_node',
        parameters=[{
            'delay': LaunchConfiguration('delay')
        }],
        output='screen'
    )
    
    # 返回启动描述
    return LaunchDescription([
        delay_arg,
        active_questioning_node
    ])
