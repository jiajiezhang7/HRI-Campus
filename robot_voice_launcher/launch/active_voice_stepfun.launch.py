#!/usr/bin/env python3

"""
集成启动文件，先启动语音系统，然后再启动主动发问节点并自动触发服务
"""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, 
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    """生成启动描述，先启动语音系统，然后再启动主动发问节点并自动触发服务"""
    
    # 查找包的路径
    robot_voice_pkg_dir = FindPackageShare('robot_voice_launcher')
    
    # 语音系统启动文件
    voice_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([robot_voice_pkg_dir, 'launch', 'voice_system_stepfun.launch.py'])
        ]),
        # 这里可以添加launch参数覆盖，例如:
        # launch_arguments={'enable_filter': 'true'}.items()
    )
    
    # 服务触发器节点
    service_trigger_node = Node(
        package='robot_voice_launcher',
        executable='service_trigger_node',
        name='service_trigger_node',
        parameters=[{
            'service_name': '/active_questioning/trigger_question',
            'delay': 1.0  # 启动后3秒触发服务
        }],
        output='screen'
    )
    
    
    # 再延迟2秒启动服务触发器节点
    service_trigger_timer = TimerAction(
        period=5.0,  
        actions=[service_trigger_node]
    )
    
    # 返回启动描述
    return LaunchDescription([
        voice_system_launch,
        service_trigger_timer
    ])
