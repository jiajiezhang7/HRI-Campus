#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
测试语音状态动画显示启动文件
启动：
1. rosbridge_server 用于网页通信
2. mock_tts_status 模拟TTS状态发布节点（每8秒切换一次状态）
3. 在第二屏幕自动打开浏览器显示动画界面
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """生成启动描述"""
    
    # 获取功能包共享目录路径
    package_share_dir = FindPackageShare('robot_animation_display')
    web_dir = PathJoinSubstitution([package_share_dir, 'web'])
    html_path = PathJoinSubstitution([web_dir, 'character_display.html'])
    
    # rosbridge_server节点
    rosbridge_server_node = ExecuteProcess(
        cmd=['ros2', 'run', 'rosbridge_server', 'rosbridge_websocket'],
        name='rosbridge_websocket',
        output='screen',
    )
    
    # 模拟TTS状态发布节点
    mock_tts_status_node = Node(
        package='robot_animation_display',
        executable='mock_tts_status',
        name='mock_tts_status_publisher',
        output='screen',
    )
    
    # 浏览器启动命令（在第二屏幕上打开）
    # 使用DISPLAY=:0.1指定第二个显示器
    browser_cmd = ExecuteProcess(
        cmd=['bash', '-c', 'sleep 2 && DISPLAY=:0.1 firefox --new-window "file://$(ros2 pkg prefix robot_animation_display)/share/robot_animation_display/web/character_display.html" --kiosk'],
        name='browser',
        output='screen',
    )
    
    # 先启动rosbridge和模拟节点，然后再启动浏览器
    return LaunchDescription([
        rosbridge_server_node,
        mock_tts_status_node,
        # 延迟2秒启动浏览器，确保其他节点已启动
        TimerAction(
            period=2.0,
            actions=[browser_cmd]
        ),
    ])
