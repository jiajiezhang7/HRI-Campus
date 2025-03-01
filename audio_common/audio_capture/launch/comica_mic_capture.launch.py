#!/usr/bin/env python3

"""
Launch file for capturing audio from Comica_VM10 PRO microphone
and publishing it to a ROS2 topic.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    """Generate launch description for audio capture node."""
    
    # 声明所有launch参数
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='plughw:3,0',  # 使用ALSA插件接口而不是直接硬件访问
        description='Audio device identifier'
    )
    
    format_arg = DeclareLaunchArgument(
        'format',
        default_value='wave',  # 修正：使用'wave'而不是'wav'
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
    
    # 创建音频捕获节点
    audio_capture_node = Node(
        package='audio_capture',
        executable='audio_capture_node',
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
        group_action
    ])
