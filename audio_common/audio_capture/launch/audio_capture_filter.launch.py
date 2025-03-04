#!/usr/bin/env python3

"""
Launch file for capturing audio from microphone with high-pass filter
and publishing it to a ROS2 topic.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    """Generate launch description for audio capture node with filter."""
    
    # 声明所有launch参数
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='plughw:3,0',  # 使用ALSA插件接口而不是直接硬件访问
        description='Audio device identifier'
    )
    
    format_arg = DeclareLaunchArgument(
        'format',
        default_value='wave',  # 使用'wave'格式以便进行滤波处理
        description='Audio format (wave or mp3 only)'
    )
    
    bitrate_arg = DeclareLaunchArgument(
        'bitrate',
        default_value='128',
        description='Audio bitrate'
    )
    
    channels_arg = DeclareLaunchArgument(
        'channels',
        default_value='2',  # 修改为2个声道（立体声），设备不支持单声道
        description='Number of audio channels (1 for mono, 2 for stereo)'
    )
    
    depth_arg = DeclareLaunchArgument(
        'depth',
        default_value='16',  # 16位深度
        description='Audio bit depth'
    )
    
    sample_rate_arg = DeclareLaunchArgument(
        'sample_rate',
        default_value='16000',  # 修改为16kHz，与语音识别匹配
        description='Audio sampling rate in Hz'
    )
    
    sample_format_arg = DeclareLaunchArgument(
        'sample_format',
        default_value='S16LE',  # 16位有符号小端格式
        description='Audio sample format'
    )
    
    dst_arg = DeclareLaunchArgument(
        'dst',
        default_value='appsink',
        description='Audio destination'
    )
    
    ns_arg = DeclareLaunchArgument(
        'ns',
        default_value='audio',
        description='Namespace for the audio nodes'
    )
    
    audio_topic_arg = DeclareLaunchArgument(
        'audio_topic',
        default_value='audio',
        description='Name of the audio topic'
    )
    
    # 高通滤波器参数
    enable_filter_arg = DeclareLaunchArgument(
        'enable_filter',
        default_value='false',  # 默认启用滤波器
        description='Enable high-pass filter for noise reduction'
    )
    
    cutoff_frequency_arg = DeclareLaunchArgument(
        'cutoff_frequency',
        default_value='100.0',  # 默认截止频率100Hz
        description='Cutoff frequency for high-pass filter in Hz'
    )
    
    # 创建音频捕获节点（带滤波器）
    audio_capture_node = Node(
        package='audio_capture',
        executable='audio_capture_filter_node',  # 使用新的可执行文件
        name='audio_capture',
        output='screen',
        parameters=[{
            'dst': LaunchConfiguration('dst'),
            'device': LaunchConfiguration('device'),
            'format': LaunchConfiguration('format'),
            'bitrate': LaunchConfiguration('bitrate'),
            'channels': LaunchConfiguration('channels'),
            'depth': LaunchConfiguration('depth'),
            'sample_rate': LaunchConfiguration('sample_rate'),
            'sample_format': LaunchConfiguration('sample_format'),
            'enable_filter': LaunchConfiguration('enable_filter'),
            'cutoff_frequency': LaunchConfiguration('cutoff_frequency'),
        }],
        remappings=[
            ('audio', LaunchConfiguration('audio_topic')),
        ]
    )
    
    # 将节点放在指定的命名空间中
    group_action = GroupAction([
        PushRosNamespace(LaunchConfiguration('ns')),
        audio_capture_node
    ])
    
    # 返回LaunchDescription对象
    return LaunchDescription([
        device_arg,
        format_arg,
        bitrate_arg,
        channels_arg,
        depth_arg,
        sample_rate_arg,
        sample_format_arg,
        dst_arg,
        ns_arg,
        audio_topic_arg,
        enable_filter_arg,
        cutoff_frequency_arg,
        group_action
    ])
