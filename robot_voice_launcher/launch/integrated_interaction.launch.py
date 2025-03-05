#!/usr/bin/env python3

"""
集成交互启动文件，启动摄像头系统和语音交互系统
"""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, 
    ExecuteProcess,
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    """生成启动描述，启动摄像头系统和语音交互系统"""
    
    # 查找包的路径
    robot_voice_pkg_dir = FindPackageShare('robot_voice_launcher')
    
    # 摄像头系统启动文件
    camera_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([robot_voice_pkg_dir, 'launch', 'camera_system.launch.py'])
        ]),
    )
    
    # 语音系统启动文件（使用stepfun接口）
    voice_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([robot_voice_pkg_dir, 'launch', 'voice_system_stepfun.launch.py'])
        ]),
    )
    
    # 交互协调器节点
    interaction_coordinator_node = Node(
        package='robot_voice_launcher',
        executable='interaction_coordinator_node',
        name='interaction_coordinator_node',
        output='screen'
    )
    
    # 返回启动描述
    return LaunchDescription([
        # 先启动摄像头系统
        camera_system_launch,
        
        # 等待3秒后启动语音系统
        TimerAction(
            period=3.0,
            actions=[voice_system_launch]
        ),
        
        # 等待8秒后启动交互协调器节点
        TimerAction(
            period=8.0,
            actions=[interaction_coordinator_node]
        )
    ])
