#!/usr/bin/env python3

"""
主启动文件，按顺序启动：
1. audio_capture_filter.launch.py (带高通滤波器的麦克风捕获)
2. speech_recognition_stepfun
3. llm_bytedance_general (通用对话系统)
4. speech_generation_stepfun
5. audio_play_python
"""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    DeclareLaunchArgument,
    GroupAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

def generate_launch_description():
    """生成启动描述，按顺序启动所有语音相关节点"""

    # 声明是否启用高通滤波器的参数
    enable_filter_arg = DeclareLaunchArgument(
        'enable_filter',
        default_value='true',
        description='Enable high-pass filter for noise reduction'
    )

    # 声明高通滤波器截止频率参数
    cutoff_frequency_arg = DeclareLaunchArgument(
        'cutoff_frequency',
        default_value='800.0',
        description='Cutoff frequency for high-pass filter in Hz'
    )

    # 声明语音识别静音阈值参数
    silence_threshold_arg = DeclareLaunchArgument(
        'silence_threshold',
        default_value='50',
        description='Threshold for silence detection in speech recognition'
    )
    
    # 声明LLM类型参数，用于选择使用原始LLM节点还是LangChain节点
    use_langchain_arg = DeclareLaunchArgument(
        'use_langchain',
        default_value='true',
        description='使用LangChain节点(true)还是原始LLM节点(false)'
    )

    # 查找各个包的路径
    audio_capture_pkg_dir = FindPackageShare('audio_capture')
    speech_recognition_pkg_dir = FindPackageShare('speech_recognition_stepfun')
    llm_bytedance_pkg_dir = FindPackageShare('llm_bytedance')
    speech_generation_pkg_dir = FindPackageShare('speech_generation_stepfun')
    audio_play_pkg_dir = FindPackageShare('audio_play_python')

    # 麦克风捕获启动文件（带高通滤波器）
    mic_capture_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([audio_capture_pkg_dir, 'launch', 'audio_capture_filter.launch.py'])
        ]),
        launch_arguments={
            'enable_filter': LaunchConfiguration('enable_filter'),
            'cutoff_frequency': LaunchConfiguration('cutoff_frequency')
        }.items()
    )

    # 语音识别启动文件（使用阶跃星辰API）
    speech_recognition_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([speech_recognition_pkg_dir, 'launch', 'speech_recognition_stepfun.launch.py'])
        ]),
        launch_arguments={
            'silence_threshold': LaunchConfiguration('silence_threshold')
        }.items()
    )

    # 使用原始LLM节点的启动文件
    llm_bytedance_general = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([llm_bytedance_pkg_dir, 'launch', 'llm_bytedance_general.launch.py'])
        ]),
        condition=UnlessCondition(LaunchConfiguration('use_langchain'))
    )
    
    # 使用LangChain节点的启动文件
    llm_langchain = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([llm_bytedance_pkg_dir, 'launch', 'langchain_node.launch.py'])
        ]),
        condition=IfCondition(LaunchConfiguration('use_langchain'))
    )

    # 语音合成启动文件（使用阶跃星辰API）
    speech_generation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([speech_generation_pkg_dir, 'launch', 'speech_generation_stepfun.launch.py'])
        ]),
        # 这里可以添加launch参数覆盖，例如:
        # launch_arguments={'voice': 'cixingnansheng'}.items()
    )

    # 音频播放启动文件
    audio_play_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([audio_play_pkg_dir, 'launch', 'audio_play.launch.py'])
        ]),
        launch_arguments={
            'format': 'mp3',
            'device': 'plughw:1,0'  # 使用Philips SPA2100扬声器
        }.items()
    )

    # 麦克风静音控制节点
    mic_mute_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('robot_voice_launcher'), 'launch', 'mic_mute.launch.py'])
        ])
    )

    # 使用通用对话系统的主动发问节点
    active_questioning_general = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('robot_voice_launcher'), 'launch', 'active_questioning_general.launch.py'])
        ])
    )

    # 音频录制节点已禁用
    # audio_recorder_script = os.path.join('/home/agilex03/hri_ws/src', 'audio_recorder.py')
    # audio_recorder_node = ExecuteProcess(
    #     cmd=['python3', audio_recorder_script],
    #     name='audio_recorder',
    #     output='screen',
    # )

    # 按顺序启动所有节点，减少延时提高实时性
    return LaunchDescription([
        # 参数
        enable_filter_arg,
        cutoff_frequency_arg,
        silence_threshold_arg,
        use_langchain_arg,

        # 麦克风捕获节点
        mic_capture_launch,

        # 等待0.5秒后启动语音识别（确保麦克风初始化完成）
        TimerAction(
            period=0.5,
            actions=[speech_recognition_launch]
        ),

        # 等待0.8秒后启动通用对话系统LLM（根据参数选择原始或LangChain节点）
        TimerAction(
            period=0.8,
            actions=[llm_bytedance_general, llm_langchain]
        ),

        # 等待1.0秒后启动语音合成
        TimerAction(
            period=1.0,
            actions=[speech_generation_launch]
        ),

        # 等待1.2秒后启动音频播放
        TimerAction(
            period=1.2,
            actions=[audio_play_launch]
        ),

        # 等待1.3秒后启动麦克风静音控制
        TimerAction(
            period=1.3,
            actions=[mic_mute_launch]
        ),

        # 等待1.5秒后启动通用对话系统的主动发问节点
        TimerAction(
            period=1.5,
            actions=[active_questioning_general]
        ),

        # 等待1.5秒后启动音频记录器（可选）
        # TimerAction(
        #     period=1.5,
        #     actions=[audio_recorder_node]
        # )
    ])
