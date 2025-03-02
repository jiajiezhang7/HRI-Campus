#!/usr/bin/env python3

"""
Launch file for Baidu speech recognition system
Launches both audio capture and speech recognition nodes
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """Generate launch description for Baidu speech recognition system."""
    
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
    
    # 百度API参数
    baidu_api_key_arg = DeclareLaunchArgument(
        'baidu_api_key',
        default_value='pwPe748UolTnzuxecNcwnOvj',
        description='百度语音识别API Key'
    )
    
    baidu_secret_key_arg = DeclareLaunchArgument(
        'baidu_secret_key',
        default_value='Fm5Ntlb4FaC2bUy3vgsLDCftRasaAFDM',
        description='百度语音识别Secret Key'
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
        package='speech_recognition_baidu',
        executable='speech_recognition_baidu_node',
        name='speech_recognition_baidu',
        output='screen',
        parameters=[{
            'language': LaunchConfiguration('language'),
            'buffer_size': LaunchConfiguration('buffer_size'),
            'silence_threshold': LaunchConfiguration('silence_threshold'),
            'min_utterance_length': LaunchConfiguration('min_utterance_length'),
            'silence_duration': LaunchConfiguration('silence_duration'),
            'baidu_api_key': LaunchConfiguration('baidu_api_key'),
            'baidu_secret_key': LaunchConfiguration('baidu_secret_key'),
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
        baidu_api_key_arg,
        baidu_secret_key_arg,
        use_proxy_arg,
        http_proxy_arg,
        https_proxy_arg,
        audio_capture_launch,
        speech_recognition_node
    ])
