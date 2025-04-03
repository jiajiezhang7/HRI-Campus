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
        description='延迟发问的时间（秒）'
    )
    
    # 声明场景类型参数
    scene_arg = DeclareLaunchArgument(
        'scene',
        default_value='greeting',  # 默认为问候场景
        description='场景类型，可选值：greeting, direction, information, help'
    )
    
    # 声明检查订阅者参数
    check_subscribers_arg = DeclareLaunchArgument(
        'check_subscribers',
        default_value='true',  # 默认检查订阅者
        description='是否检查订阅者'
    )
    
    # 声明最大重试次数参数
    max_retries_arg = DeclareLaunchArgument(
        'max_retries',
        default_value='10',  # 默认最大重试10次
        description='最大重试次数'
    )
    
    # 声明重试间隔参数
    retry_interval_arg = DeclareLaunchArgument(
        'retry_interval',
        default_value='1.0',  # 默认重试间隔1秒
        description='重试间隔（秒）'
    )
    
    # 主动发问节点
    active_questioning_node = Node(
        package='robot_voice_launcher',
        executable='active_questioning_general_node',
        name='active_questioning_general_node',
        parameters=[{
            'delay': LaunchConfiguration('delay'),
            'default_scene': LaunchConfiguration('scene'),
            'check_subscribers': LaunchConfiguration('check_subscribers'),
            'max_retries': LaunchConfiguration('max_retries'),
            'retry_interval': LaunchConfiguration('retry_interval')
        }],
        output='screen'
    )
    
    # 返回启动描述
    return LaunchDescription([
        delay_arg,
        scene_arg,
        check_subscribers_arg,
        max_retries_arg,
        retry_interval_arg,
        active_questioning_node
    ])
