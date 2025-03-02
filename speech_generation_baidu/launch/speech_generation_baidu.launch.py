from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明启动参数
    baidu_api_key_arg = DeclareLaunchArgument(
        'baidu_api_key',
        default_value='pwPe748UolTnzuxecNcwnOvj',
        description='百度语音API的API Key'
    )
    
    baidu_secret_key_arg = DeclareLaunchArgument(
        'baidu_secret_key',
        default_value='Fm5Ntlb4FaC2bUy3vgsLDCftRasaAFDM',
        description='百度语音API的Secret Key'
    )
    
    # 发音人选择, 基础音库：0为度小美，1为度小宇，3为度逍遥，4为度丫丫，
    # 精品音库：5为度小娇，103为度米朵，106为度博文，110为度小童，111为度小萌，默认为度小美 
    per_arg = DeclareLaunchArgument(
        'per',
        default_value='4226',
        description='发音人选择'


    )
    
    spd_arg = DeclareLaunchArgument(
        'spd',
        default_value='5',
        description='语速，取值0-15，默认为5中语速'
    )
    
    pit_arg = DeclareLaunchArgument(
        'pit',
        default_value='5',
        description='音调，取值0-15，默认为5中音调'
    )
    
    vol_arg = DeclareLaunchArgument(
        'vol',
        default_value='5',
        description='音量，取值0-9，默认为5中音量'
    )
    
    aue_arg = DeclareLaunchArgument(
        'aue',
        default_value='3',
        description='文件格式，3:mp3, 4:pcm-16k, 5:pcm-8k, 6:wav'
    )
    
    # 创建节点
    node = Node(
        package='speech_generation_baidu',
        executable='speech_generation_baidu_node',
        name='speech_generation_baidu_node',
        output='screen',
        parameters=[{
            'baidu_api_key': LaunchConfiguration('baidu_api_key'),
            'baidu_secret_key': LaunchConfiguration('baidu_secret_key'),
            'per': LaunchConfiguration('per'),
            'spd': LaunchConfiguration('spd'),
            'pit': LaunchConfiguration('pit'),
            'vol': LaunchConfiguration('vol'),
            'aue': LaunchConfiguration('aue'),
        }]
    )
    
    return LaunchDescription([
        baidu_api_key_arg,
        baidu_secret_key_arg,
        per_arg,
        spd_arg,
        pit_arg,
        vol_arg,
        aue_arg,
        node
    ])
