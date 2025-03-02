#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, EnvironmentVariable


def generate_launch_description():
    """生成启动描述"""
    
    # 声明启动参数
    model_id_arg = DeclareLaunchArgument(
        'model_id',
        default_value='ep-20250213183336-ngs4d',
        description='火山引擎大模型-DeepSeek-V3-ID'
    )
    
    # 设置环境变量
    api_key = SetEnvironmentVariable(
        name='ARK_API_KEY',
        value=EnvironmentVariable('ARK_API_KEY', default_value='8967f487-1a84-4340-8ac5-79f087456b95')
    )
    
    model_id = SetEnvironmentVariable(
        name='ARK_MODEL_ID',
        value=LaunchConfiguration('model_id')
    )
    
    # 创建节点
    llm_bytedance_node = Node(
        package='llm_bytedance',
        executable='llm_bytedance_node',
        name='llm_bytedance_node',
        output='screen',
        emulate_tty=True,
        parameters=[]
    )
    
    # 返回LaunchDescription
    return LaunchDescription([
        model_id_arg,
        api_key,
        model_id,
        llm_bytedance_node
    ])
