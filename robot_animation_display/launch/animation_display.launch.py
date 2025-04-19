#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
语音状态动画显示启动文件
启动：
1. rosbridge_server 用于网页通信
2. tts_status_publisher 发布TTS状态
3. 在第二屏幕自动打开浏览器显示动画界面
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """生成启动描述"""
    
    # 获取功能包共享目录路径
    package_share_dir = FindPackageShare('robot_animation_display')
    web_dir = PathJoinSubstitution([package_share_dir, 'web'])
    html_path = PathJoinSubstitution([web_dir, 'character_display.html'])
    
    # 添加rosbridge端口参数
    rosbridge_port_arg = DeclareLaunchArgument(
        'rosbridge_port',
        default_value='9090',
        description='Port for rosbridge websocket'
    )
    
    # rosbridge_server节点
    rosbridge_server_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'port': LaunchConfiguration('rosbridge_port')}]
    )
    
    # TTS状态发布节点
    tts_status_publisher_node = Node(
        package='robot_animation_display',
        executable='tts_status_publisher',
        name='tts_status_publisher',
        output='screen',
    )
    
    # 浏览器启动命令（尝试使用Chrome打开）
    # 使用--window-position参数将窗口定位到HDMI-1显示器上
    browser_cmd = ExecuteProcess(
        cmd=['bash', '-c', 'sleep 2 && google-chrome --new-window "file://$(ros2 pkg prefix robot_animation_display)/share/robot_animation_display/web/character_display.html" --start-fullscreen --window-position=1920,0'],
        name='browser',
        output='screen',
    )
    
    # 先启动rosbridge和TTS状态发布节点，然后再启动浏览器
    return LaunchDescription([
        rosbridge_port_arg,
        rosbridge_server_node,
        tts_status_publisher_node,
        # 延迟2秒启动浏览器，确保其他节点已启动
        TimerAction(
            period=2.0,
            actions=[browser_cmd]
        ),
    ])
