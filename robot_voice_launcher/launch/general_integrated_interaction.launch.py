#!/usr/bin/env python3


"""
总入口：集成交互启动文件，启动摄像头系统和通用对话系统
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
    """生成启动描述，启动摄像头系统、通用对话系统和动画显示系统"""

    # 查找包的路径
    robot_voice_pkg_dir = FindPackageShare('robot_voice_launcher')
    robot_animation_pkg_dir = FindPackageShare('robot_animation_display')

    # 摄像头系统启动文件
    camera_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([robot_voice_pkg_dir, 'launch', 'camera_system.launch.py'])
        ]),
    )

    # 语音系统启动文件（使用stepfun接口，通用对话系统）
    voice_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([robot_voice_pkg_dir, 'launch', 'general_voice_system_stepfun.launch.py'])
        ]),
    )

    # 动画显示启动文件
    animation_display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([robot_animation_pkg_dir, 'launch', 'animation_display.launch.py'])
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
        # 先启动动画显示系统
        animation_display_launch,

        # 等待1秒后启动摄像头系统
        TimerAction(
            period=1.0,
            actions=[
                camera_system_launch
            ]
        ),

        # 等待3秒后启动语音系统（确保动画系统和摄像头系统已启动）
        TimerAction(
            period=3.0,
            actions=[voice_system_launch]
        ),

        # 等待5秒后启动交互协调器节点（确保语音系统已启动）
        TimerAction(
            period=7.0,
            actions=[interaction_coordinator_node]
        )
    ])
