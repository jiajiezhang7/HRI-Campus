#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 包路径
    robot_voice_launcher_dir = get_package_share_directory('robot_voice_launcher')
    nav_dialogue_integration_dir = get_package_share_directory('nav_dialogue_integration')
    
    # 初始位姿参数 - 只需在这里修改默认值即可全局生效
    initial_pose_x_default = '-4.234'
    initial_pose_y_default = '0.306'
    initial_pose_theta_default = '2.925'
    
    # 场景类型参数
    scene_type = LaunchConfiguration('scene_type', default='general')
    
    # 是否启动导航系统
    start_navigation = LaunchConfiguration('start_navigation', default='true')
    
    # 初始位姿参数配置
    initial_pose_x = LaunchConfiguration('initial_pose_x', default=initial_pose_x_default)
    initial_pose_y = LaunchConfiguration('initial_pose_y', default=initial_pose_y_default)
    initial_pose_theta = LaunchConfiguration('initial_pose_theta', default=initial_pose_theta_default)
    
    # 配置文件路径
    locations_config = LaunchConfiguration(
        'locations_config', 
        default=os.path.join(nav_dialogue_integration_dir, 'config', 'target_locations.yaml')
    )
    
    # 声明启动参数
    declare_start_navigation = DeclareLaunchArgument(
        'start_navigation',
        default_value='true',
        description='Whether to start the navigation system'
    )
    
    # 声明初始位姿参数
    declare_initial_pose_x = DeclareLaunchArgument(
        'initial_pose_x',
        default_value=initial_pose_x_default,
        description='Initial robot pose X coordinate'
    )
    
    declare_initial_pose_y = DeclareLaunchArgument(
        'initial_pose_y',
        default_value=initial_pose_y_default,
        description='Initial robot pose Y coordinate'
    )
    
    declare_initial_pose_theta = DeclareLaunchArgument(
        'initial_pose_theta',
        default_value=initial_pose_theta_default,
        description='Initial robot pose theta (yaw) in radians'
    )
    
    # 启动导航系统
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('testzz'), 'launch', 'nav2_smac.launch.py')
        ),
        condition=IfCondition(start_navigation)
    )
    
    # 启动现有的对话系统
    general_interaction_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_voice_launcher_dir, 'launch', 'general_integrated_interaction.launch.py')
        ),
        launch_arguments={
            'scene_type': scene_type
        }.items()
    )
    
    # 启动导航对话桥接节点
    nav_dialogue_bridge_node = Node(
        package='nav_dialogue_integration',
        executable='nav_dialogue_bridge',
        name='nav_dialogue_bridge',
        output='screen',
        parameters=[
            {'locations_config': locations_config},
            {'initial_pose_x': initial_pose_x},
            {'initial_pose_y': initial_pose_y},
            {'initial_pose_theta': initial_pose_theta}
        ]
    )
    
    # 设置日志级别为WARN
    log_level_warn = SetEnvironmentVariable(
        name='RCUTILS_LOGGING_LEVEL',
        value='WARN'
    )
    
    # 将所有节点组合为一个启动描述
    return LaunchDescription([
        # 首先设置日志级别
        log_level_warn,
        # 然后声明其他启动参数
        declare_start_navigation,
        declare_initial_pose_x,
        declare_initial_pose_y,
        declare_initial_pose_theta,
        navigation_launch,
        general_interaction_launch,
        nav_dialogue_bridge_node
    ])
