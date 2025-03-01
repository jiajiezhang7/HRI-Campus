#!/usr/bin/env python3

"""
Launch file for speech recognition system
Launches both audio capture and speech recognition nodes
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """Generate launch description for speech recognition system."""
    
    # 声明参数
    audio_ns_arg = DeclareLaunchArgument(
        'audio_ns',
        default_value='audio',
        description='Namespace for audio nodes'
    )
    
    language_arg = DeclareLaunchArgument(
        'language',
        default_value='zh-cn',
        description='Language for speech recognition (en-us or zh-cn)'
    )
    
    buffer_size_arg = DeclareLaunchArgument(
        'buffer_size',
        default_value='32000',
        description='Audio buffer size for speech recognition'
    )
    
    silence_threshold_arg = DeclareLaunchArgument(
        'silence_threshold',
        default_value='200',
        description='Threshold for silence detection'
    )
    
    min_utterance_length_arg = DeclareLaunchArgument(
        'min_utterance_length',
        default_value='16000',
        description='Minimum length of utterance to recognize (in bytes)'
    )
    
    silence_duration_arg = DeclareLaunchArgument(
        'silence_duration',
        default_value='1.5',
        description='Duration of silence to consider end of utterance (in seconds)'
    )
    
    # 科大讯飞API参数 - 已设置默认值为您提供的API密钥
    xf_app_id_arg = DeclareLaunchArgument(
        'xf_app_id',
        default_value='c0ffa3f9',   # 64bb7d13
        description='科大讯飞应用ID'
    )
    
    xf_api_key_arg = DeclareLaunchArgument(
        'xf_api_key',
        default_value='400d056c258a53e030c80d2ee1c081a5',  #75ee9180e28e4de40ecbde29b57a58bc
        description='科大讯飞API密钥'
    )
    
    xf_api_secret_arg = DeclareLaunchArgument(
        'xf_api_secret',
        default_value='MjRkMTFkODQ2MDk1MDA2OTZmNDJiZTUw',  #NmE1MWM5NzZjYWEyM2ExODE1ZmUxNmRj
        description='科大讯飞API Secret'
    )
    
    # 网络代理参数
    use_proxy_arg = DeclareLaunchArgument(
        'use_proxy',
        default_value='false',
        description='是否使用代理服务器连接API'
    )
    
    http_proxy_arg = DeclareLaunchArgument(
        'http_proxy',
        default_value='',
        description='HTTP代理服务器地址 (例如: http://proxy.example.com:8080)'
    )
    
    https_proxy_arg = DeclareLaunchArgument(
        'https_proxy',
        default_value='',
        description='HTTPS代理服务器地址 (例如: https://proxy.example.com:8080)'
    )
    
    # 包含音频捕获launch文件
    audio_capture_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('audio_capture'),
                'launch',
                'comica_mic_capture.launch.py'
            ])
        ])
    )
    
    # 创建语音识别节点
    speech_recognition_node = Node(
        package='speech_recognition',
        executable='speech_recognition_node',
        name='speech_recognition',
        output='screen',
        parameters=[{
            'language': LaunchConfiguration('language'),
            'buffer_size': LaunchConfiguration('buffer_size'),
            'silence_threshold': LaunchConfiguration('silence_threshold'),
            'min_utterance_length': LaunchConfiguration('min_utterance_length'),
            'silence_duration': LaunchConfiguration('silence_duration'),
            'xf_app_id': LaunchConfiguration('xf_app_id'),
            'xf_api_key': LaunchConfiguration('xf_api_key'),
            'xf_api_secret': LaunchConfiguration('xf_api_secret'),
            'use_proxy': LaunchConfiguration('use_proxy'),
            'http_proxy': LaunchConfiguration('http_proxy'),
            'https_proxy': LaunchConfiguration('https_proxy'),
        }]
    )
    
    # 返回LaunchDescription对象
    return LaunchDescription([
        audio_ns_arg,
        language_arg,
        buffer_size_arg,
        silence_threshold_arg,
        min_utterance_length_arg,
        silence_duration_arg,
        xf_app_id_arg,
        xf_api_key_arg, 
        xf_api_secret_arg,
        use_proxy_arg,
        http_proxy_arg,
        https_proxy_arg,
        audio_capture_launch,
        speech_recognition_node
    ])
