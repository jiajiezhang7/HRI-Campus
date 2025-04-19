#!/usr/bin/env python3

"""
启动电梯交互FlexBE行为
"""

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
    DeclareLaunchArgument
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 查找包路径
    robot_voice_launcher_share = FindPackageShare('robot_voice_launcher')
    robot_animation_display_share = FindPackageShare('robot_animation_display')

    # 启动文件路径
    camera_system_launch = PathJoinSubstitution([robot_voice_launcher_share, 'launch', 'camera_system.launch.py'])
    voice_system_launch = PathJoinSubstitution([robot_voice_launcher_share, 'launch', 'voice_system_stepfun.launch.py'])
    animation_display_launch = PathJoinSubstitution([robot_animation_display_share, 'launch', 'animation_display.launch.py'])

    # 创建启动描述
    ld = LaunchDescription()

    # 添加参数
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    ))

    # 添加rosbridge端口参数
    ld.add_action(DeclareLaunchArgument(
        'rosbridge_port',
        default_value='9092',
        description='Port for rosbridge websocket'
    ))

    # 启动动画显示系统
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([animation_display_launch]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rosbridge_port': LaunchConfiguration('rosbridge_port')
        }.items()
    ))

    # 等待1秒后启动摄像头系统和电梯信息发布节点
    ld.add_action(TimerAction(
        period=1.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([camera_system_launch]),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            ),
            Node(
                package='dummy_level_publisher',
                executable='dummy_level_publisher',
                name='dummy_level_publisher_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'publish_frequency': 5.0}
                ]
            )
        ]
    ))

    # 等待3秒后启动语音系统
    ld.add_action(TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([voice_system_launch]),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    ))

    # 等待7秒后启动FlexBE系统
    ld.add_action(TimerAction(
        period=7.0,
        actions=[
            # 启动FlexBE Onboard
            Node(
                package='flexbe_onboard',
                executable='start_behavior',
                name='behavior_onboard',
                parameters=[{'use_sim_time': use_sim_time}]
            ),
            # 启动FlexBE Mirror
            Node(
                package='flexbe_mirror',
                executable='behavior_mirror_sm',
                name='behavior_mirror',
                parameters=[{'use_sim_time': use_sim_time}]
            ),
            # 启动FlexBE UI Server
            Node(
                package='flexbe_webui',
                executable='webui_node',
                name='webui_node',
                parameters=[{'port': 8000}]  # 使用默认端口
            ),
            # 延迟打开浏览器，确保UI服务已启动
            TimerAction(
                period=2.0,
                actions=[
                    ExecuteProcess(
                        cmd=['python3', '-m', 'webbrowser', '-n', 'http://127.0.0.1:8000'],
                        name='open_flexbe_webui'
                    )
                ]
            ),
            # 启动FlexBE行为
            Node(
                package='flexbe_widget',
                executable='be_launcher',
                name='behavior_launcher',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=['-b', 'Elevator Interaction Behavior']
            )
        ]
    ))

    return ld
