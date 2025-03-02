#!/usr/bin/env python3

"""
主启动文件，按顺序启动：
1. comica_mic_capture.launch.py
2. speech_recognition_baidu
3. llm_bytedance
4. speech_generation_baidu
5. audio_recorder.py
"""

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, 
    ExecuteProcess,
    GroupAction,
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    """生成启动描述，按顺序启动所有语音相关节点"""
    
    # 查找各个包的路径
    audio_capture_pkg_dir = FindPackageShare('audio_capture')
    speech_recognition_pkg_dir = FindPackageShare('speech_recognition_baidu')
    llm_bytedance_pkg_dir = FindPackageShare('llm_bytedance')
    speech_generation_pkg_dir = FindPackageShare('speech_generation_baidu')
    
    # 麦克风捕获启动文件
    mic_capture_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([audio_capture_pkg_dir, 'launch', 'comica_mic_capture.launch.py'])
        ])
    )
    
    # 语音识别启动文件
    speech_recognition_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([speech_recognition_pkg_dir, 'launch', 'speech_recognition_baidu.launch.py'])
        ]),
        # 这里可以添加launch参数覆盖，例如:
        # launch_arguments={'parameter_name': 'new_value'}.items()
    )
    
    # LLM字节跳动启动
    llm_bytedance_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([llm_bytedance_pkg_dir, 'launch', 'llm_bytedance.launch.py'])
        ]),
        # 这里可以添加launch参数覆盖，例如:
        # launch_arguments={'parameter_name': 'new_value'}.items()
    )
    
    # 语音合成启动文件
    speech_generation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([speech_generation_pkg_dir, 'launch', 'speech_generation_baidu.launch.py'])
        ]),
        # 这里可以添加launch参数覆盖，例如:
        # launch_arguments={'per': '106'}.items()
    )
    
    # 音频录制节点
    audio_recorder_script = os.path.join('/home/jay/microp_ws/src', 'audio_recorder.py')
    audio_recorder_node = ExecuteProcess(
        cmd=['python3', audio_recorder_script],
        name='audio_recorder',
        output='screen',
    )
    
    # 使用TimerAction实现顺序启动
    # 先启动麦克风捕获
    mic_capture_group = GroupAction([mic_capture_launch])
    
    # 2秒后启动语音识别
    recognition_timer = TimerAction(
        period=2.0,
        actions=[speech_recognition_launch]
    )
    
    # 4秒后启动LLM节点
    llm_timer = TimerAction(
        period=4.0,
        actions=[llm_bytedance_launch]
    )
    
    # 6秒后启动语音合成
    speech_gen_timer = TimerAction(
        period=6.0,
        actions=[speech_generation_launch]
    )
    
    # 8秒后启动音频录制
    recorder_timer = TimerAction(
        period=8.0,
        actions=[audio_recorder_node]
    )
    
    # 返回启动描述
    return LaunchDescription([
        mic_capture_group,
        recognition_timer,
        llm_timer,
        speech_gen_timer,
        recorder_timer
    ])
