#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """生成启动描述"""
    
    # 声明启动参数
    prompt_type_arg = DeclareLaunchArgument(
        'prompt_type',
        default_value='general',
        description='要使用的提示词类型，对应system_prompts.yaml中的键'
    )
    
    # 创建LangChain节点
    langchain_node = Node(
        package='llm_bytedance',
        executable='llm_bytedance_langchain_node',
        name='llm_bytedance_langchain',
        parameters=[{
            'prompt_type': LaunchConfiguration('prompt_type')
        }],
        output='screen'
    )
    
    # 返回启动描述
    return LaunchDescription([
        prompt_type_arg,
        langchain_node
    ])
