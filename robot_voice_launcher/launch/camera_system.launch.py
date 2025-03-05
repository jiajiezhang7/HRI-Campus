#!/usr/bin/env python3

"""
摄像头系统启动文件，用于启动摄像头、鱼眼处理、人体检测和舵机控制节点
"""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    """生成启动描述，启动摄像头系统的各个组件"""
    
    # 查找usb_cam包的路径
    usb_cam_pkg_dir = FindPackageShare('usb_cam')
    
    # 摄像头启动文件
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([usb_cam_pkg_dir, 'launch', 'camera.launch.py'])
        ]),
    )
    
    # 鱼眼处理节点
    fisheye_process_node = ExecuteProcess(
        cmd=['ros2', 'run', 'fisheye_process', 'fisheye_stitch'],
        output='screen'
    )
    
    # 人体检测节点
    human_detect_node = ExecuteProcess(
        cmd=['ros2', 'run', 'human_detect', 'detect'],
        output='screen'
    )
    
    # 舵机控制节点
    servo_node = ExecuteProcess(
        cmd=['ros2', 'run', 'servo', 'servo'],
        output='screen'
    )
    
    # 返回启动描述
    return LaunchDescription([
        camera_launch,
        fisheye_process_node,
        human_detect_node,
        servo_node
    ])
