#!/usr/bin/env python3

"""
集成启动文件，先启动语音系统，然后再启动主动发问节点
"""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, 
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    """生成启动描述，先启动语音系统，然后再启动主动发问节点"""
    
    # 查找包的路径
    robot_voice_pkg_dir = FindPackageShare('robot_voice_launcher')
    
    # 语音系统启动文件
    voice_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([robot_voice_pkg_dir, 'launch', 'voice_system.launch.py'])
        ]),
        # 这里可以添加launch参数覆盖，例如:
        # launch_arguments={'enable_filter': 'true'}.items()
    )
    
    # 主动发问启动文件
    active_questioning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([robot_voice_pkg_dir, 'launch', 'active_questioning.launch.py'])
        ]),
        # 这里可以添加launch参数覆盖，例如:
        launch_arguments={'delay': '3.0'}.items()  # 启动后3秒发问，给语音系统足够时间初始化
    )
    
    # 先启动语音系统，然后延迟8秒启动主动发问节点
    active_questioning_timer = TimerAction(
        period=8.0,  # 给语音系统8秒的时间完全启动
        actions=[active_questioning_launch]
    )
    
    # 返回启动描述
    return LaunchDescription([
        voice_system_launch,
        active_questioning_timer
    ])
