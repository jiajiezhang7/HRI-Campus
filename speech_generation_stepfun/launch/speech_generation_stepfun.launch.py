from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明启动参数
    stepfun_api_key_arg = DeclareLaunchArgument(
        'stepfun_api_key',
        default_value='1XNnG1CYt5H6hwNpSv9hYAqlVDZOaGoyeshIZzeOv8eLE8r5hc5WyLAM8TwahJTpd',
        description='阶跃星辰API的API Key'
    )
    
    # 音色选择
    voice_arg = DeclareLaunchArgument(
        'voice',
        default_value='cixingnansheng',
        # 优雅女生 youyanvsheng
        # 软萌女生 ruanmengnvsheng
        # 温柔女生 wenrounvsheng
        # 深沉男音 shenchennanyin
        description='生成时使用的音色信息'
    )
    
    # 响应格式
    response_format_arg = DeclareLaunchArgument(
        'response_format',
        default_value='mp3',
        description='返回的音频格式，支持 wav,mp3,flac,opus'
    )
    
    # 语速
    speed_arg = DeclareLaunchArgument(
        'speed',
        default_value='1.0',
        description='语速，取值范围为 0.5~2，默认值 1.0'
    )
    
    # 音量
    volume_arg = DeclareLaunchArgument(
        'volume',
        default_value='1.0',
        description='音量，取值范围为 0.1~2.0，默认值 1.0'
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
    
    # 创建节点
    node = Node(
        package='speech_generation_stepfun',
        executable='speech_generation_stepfun_node',
        name='speech_generation_stepfun_node',
        output='screen',
        parameters=[{
            'stepfun_api_key': LaunchConfiguration('stepfun_api_key'),
            'voice': LaunchConfiguration('voice'),
            'response_format': LaunchConfiguration('response_format'),
            'speed': LaunchConfiguration('speed'),
            'volume': LaunchConfiguration('volume'),
            'use_proxy': LaunchConfiguration('use_proxy'),
            'http_proxy': LaunchConfiguration('http_proxy'),
            'https_proxy': LaunchConfiguration('https_proxy'),
        }]
    )
    
    return LaunchDescription([
        stepfun_api_key_arg,
        voice_arg,
        response_format_arg,
        speed_arg,
        volume_arg,
        use_proxy_arg,
        http_proxy_arg,
        https_proxy_arg,
        node
    ])
