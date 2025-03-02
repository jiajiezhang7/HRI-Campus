from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明参数
    format_arg = DeclareLaunchArgument(
        'format',
        default_value='mp3',
        description='音频格式（mp3、wav等）'
    )
    
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='',
        description='音频设备，留空使用默认设备'
    )
    
    channels_arg = DeclareLaunchArgument(
        'channels',
        default_value='1',
        description='声道数'
    )
    
    sample_rate_arg = DeclareLaunchArgument(
        'sample_rate',
        default_value='16000',
        description='采样率'
    )
    
    # 创建节点
    audio_play_node = Node(
        package='audio_play_python',
        executable='audio_play_node',
        name='audio_play_node',
        parameters=[{
            'format': LaunchConfiguration('format'),
            'device': LaunchConfiguration('device'),
            'channels': LaunchConfiguration('channels'),
            'sample_rate': LaunchConfiguration('sample_rate'),
        }],
        output='screen'
    )
    
    return LaunchDescription([
        format_arg,
        device_arg,
        channels_arg,
        sample_rate_arg,
        audio_play_node
    ])
