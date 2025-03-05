#!/usr/bin/env python3

"""
Launch file for StepFun speech recognition system
Launches speech recognition node only
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for StepFun speech recognition system."""
    
    # 声明参数
    audio_ns_arg = DeclareLaunchArgument(
        'audio_ns',
        default_value='audio',
        description='Namespace for audio nodes'
    )
    
    language_arg = DeclareLaunchArgument(
        'language',
        default_value='zh-cn',
        description='Language for speech recognition (zh-cn)'
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
    
    # StepFun API参数
    stepfun_api_key_arg = DeclareLaunchArgument(
        'stepfun_api_key',
        default_value='1XNnG1CYt5H6hwNpSv9hYAqlVDZOaGoyeshIZzeOv8eLE8r5hc5WyLAM8TwahJTpd',
        description='StepFun API Key'
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
    
    # 创建语音识别节点
    speech_recognition_node = Node(
        package='speech_recognition_stepfun',
        executable='speech_recognition_stepfun_node',
        name='speech_recognition_stepfun',
        output='screen',
        parameters=[{
            'language': LaunchConfiguration('language'),
            'buffer_size': LaunchConfiguration('buffer_size'),
            'silence_threshold': LaunchConfiguration('silence_threshold'),
            'min_utterance_length': LaunchConfiguration('min_utterance_length'),
            'silence_duration': LaunchConfiguration('silence_duration'),
            'stepfun_api_key': LaunchConfiguration('stepfun_api_key'),
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
        stepfun_api_key_arg,
        use_proxy_arg,
        http_proxy_arg,
        https_proxy_arg,
        speech_recognition_node
    ])
