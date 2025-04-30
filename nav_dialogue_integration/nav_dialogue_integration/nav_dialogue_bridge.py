#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time
import json
import threading
import yaml
import os
import math
from ament_index_python.packages import get_package_share_directory

class NavDialogueBridge(Node):
    def __init__(self):
        super().__init__('nav_dialogue_bridge')
        
        # 订阅LLM响应话题，监听关键词
        self.llm_sub = self.create_subscription(
            String, 
            '/llm_response',
            self.llm_callback,
            10
        )
        
        # 创建发布者，用于发送导航完成后的响应消息
        self.response_pub = self.create_publisher(
            String,
            '/llm_response',
            10
        )
        
        # 创建发布者，用于发送对话历史
        self.history_pub = self.create_publisher(
            String,
            '/llm_conversation_history',
            10
        )
        
        # 预设目标位置（需要根据实际环境配置）
        self.target_locations = {
            "kitchen": self.create_pose(1.0, 2.0, 0.0),  # 示例坐标，需要替换为实际坐标
            "厨房": self.create_pose(1.0, 2.0, 0.0)      # 与kitchen相同的坐标
        }
        
        # 尝试加载配置文件
        self.declare_parameter('locations_config', '')
        config_file = self.get_parameter('locations_config').get_parameter_value().string_value
        if config_file:
            self.load_locations(config_file)
        
        # 添加初始位姿参数
        self.declare_parameter('initial_pose_x', 0.0)
        self.declare_parameter('initial_pose_y', 0.0)
        self.declare_parameter('initial_pose_theta', 0.0)
        
        # 导航状态和资源管理
        self.navigator = None
        self.is_navigating = False
        
        # 使用可重入的回调组来避免死锁
        self.callback_group = ReentrantCallbackGroup()
        
        # 导航完成后的回复消息
        self.arrival_messages = {
            "kitchen": "I've brought you to the kitchen. If I'm not mistaken, there should be various vegetables on the countertop. Please check if you can find what you need.",
            "厨房": "我已经带你走到厨房了，我没记错的话，厨房台面上会有许多蔬菜，你可以看看有没有你要的？"
        }
        
        # 创建定时器，延迟设置初始位姿
        self.set_initial_pose_timer = self.create_timer(2.0, self.set_initial_pose_callback)
        
        self.get_logger().info('Nav Dialogue Bridge节点已初始化')
    
    def load_locations(self, config_file):
        """从YAML配置文件加载位置信息"""
        try:
            # 检查是否是相对路径
            if not os.path.isabs(config_file):
                package_dir = get_package_share_directory('nav_dialogue_integration')
                config_file = os.path.join(package_dir, config_file)
            
            with open(config_file, 'r') as file:
                config = yaml.safe_load(file)
                locations = config.get('locations', {})
                
                for name, pose_data in locations.items():
                    self.target_locations[name] = self.create_pose(
                        pose_data.get('x', 0.0),
                        pose_data.get('y', 0.0),
                        pose_data.get('theta', 0.0)
                    )
                    
                    # 如果有中文名称，也添加对应项
                    if 'chinese_name' in pose_data:
                        chinese_name = pose_data['chinese_name']
                        self.target_locations[chinese_name] = self.target_locations[name]
                        
                        # 如果有到达消息，也一并添加
                        if 'arrival_message' in pose_data:
                            self.arrival_messages[name] = pose_data['arrival_message']
                        if 'chinese_arrival_message' in pose_data:
                            self.arrival_messages[chinese_name] = pose_data['chinese_arrival_message']
                
            self.get_logger().info(f'成功从{config_file}加载了{len(locations)}个位置配置')
        except Exception as e:
            self.get_logger().error(f'加载位置配置文件失败: {str(e)}')
    
    def create_pose(self, x, y, theta):
        """创建一个PoseStamped消息"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        return pose
    
    def llm_callback(self, msg):
        """处理LLM响应，检测关键词并触发导航"""
        if self.is_navigating:
            return
            
        response_text = msg.data
        self.get_logger().info(f'收到LLM响应: {response_text}')
        
        # 检查是否包含关键词
        for keyword in self.target_locations.keys():
            if keyword in response_text:
                self.get_logger().info(f'检测到关键词: {keyword}')
                
                # 在新线程中启动导航，避免阻塞回调
                nav_thread = threading.Thread(target=self.navigate_to, args=(keyword,))
                nav_thread.start()
                break
    
    def set_initial_pose_callback(self):
        """延迟设置初始位姿，确保导航系统已启动"""
        try:
            if self.navigator is None:
                self.navigator = BasicNavigator()
                
            # 等待导航系统启动
            self.navigator.waitUntilNav2Active()
                
            x = self.get_parameter('initial_pose_x').get_parameter_value().double_value
            y = self.get_parameter('initial_pose_y').get_parameter_value().double_value
            theta = self.get_parameter('initial_pose_theta').get_parameter_value().double_value
            
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.get_clock().now().to_msg()
            initial_pose.pose.position.x = x
            initial_pose.pose.position.y = y
            initial_pose.pose.position.z = 0.0
            
            # 设置方向四元数
            initial_pose.pose.orientation.w = math.cos(theta/2)
            initial_pose.pose.orientation.z = math.sin(theta/2)
            
            self.navigator.setInitialPose(initial_pose)
            self.get_logger().info(f'设置初始位姿: x={x}, y={y}, theta={theta}')
            
            # 只设置一次，然后取消定时器
            self.set_initial_pose_timer.cancel()
        except Exception as e:
            self.get_logger().error(f'设置初始位姿失败: {str(e)}')
    
    def _create_navigator(self):
        """创建导航器对象"""
        if self.navigator is None:
            self.navigator = BasicNavigator()
            
            # 等待Nav2启动，最多等待30秒
            start_time = time.time()
            timeout = 30.0  # 秒
            while time.time() - start_time < timeout:
                try:
                    # 非阻塞检查Nav2系统是否活动
                    amcl_active = self.navigator.get_service_client_handle(
                        'amcl/get_state').service_is_ready()
                    bt_navigator_active = self.navigator.get_service_client_handle(
                        'bt_navigator/get_state').service_is_ready()
                    
                    if amcl_active and bt_navigator_active:
                        self.get_logger().info('Nav2系统已激活')
                        return True
                except Exception as e:
                    self.get_logger().warning(f'检查Nav2活动状态时出错: {str(e)}')
                
                time.sleep(1.0)
            
            self.get_logger().error('Nav2系统在超时时间内未激活')
            return False
        return True
    
    def navigate_to(self, location_key):
        """导航到指定位置的主方法，启动一个非阻塞线程执行导航"""
        if self.is_navigating:
            self.get_logger().warn(f'已经在进行导航，忽略到 {location_key} 的导航请求')
            return
        
        # 检查目标位置是否有效
        if location_key not in self.target_locations:
            self.get_logger().error(f'未知的位置: {location_key}')
            return
            
        # 标记导航状态
        self.is_navigating = True
        self.get_logger().info(f'开始导航到 {location_key}')
        
        # 创建一个新线程执行导航任务
        nav_thread = threading.Thread(
            target=self._navigate_thread,
            args=(location_key,),
            daemon=True
        )
        nav_thread.start()
        
    def _navigate_thread(self, location_key):
        """在单独线程中执行导航任务"""
        response_msg = ""
        result = TaskResult.FAILED
        nav_success = False
        
        try:
            # 创建导航器
            if not self._create_navigator():
                self.get_logger().error('创建导航器失败')
                return
            
            # 发送导航目标
            target_pose = self.target_locations[location_key]
            self.navigator.goToPose(target_pose)
            
            # 等待导航完成
            while not self.navigator.isTaskComplete():
                # 非阻塞检查导航状态
                feedback = self.navigator.getFeedback()
                if feedback:
                    self.get_logger().info(f'剩余距离: {feedback.distance_remaining} 米')
                time.sleep(0.5)  # 避免CPU占用过高
            
            # 检查导航结果
            result = self.navigator.getResult()
            
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f'导航到 {location_key} 成功')
                if location_key in self.arrival_messages:
                    response_msg = self.arrival_messages[location_key]
                nav_success = True
            else:
                self.get_logger().info(f'导航到 {location_key} 失败，结果: {result}')
        
        except Exception as e:
            self.get_logger().error(f'导航过程中发生错误: {str(e)}')
        
        finally:
            # 清理导航器
            if self.navigator:
                # 仅关闭goalHandle而不完全摧毁导航器节点
                try:
                    self.navigator.cancelTask()
                except Exception as e:
                    self.get_logger().warning(f'取消导航任务时出错: {str(e)}')
            
            # 发送导航完成回复
            if response_msg and nav_success:
                self.response_pub.publish(String(data=response_msg))
                self.add_to_conversation_history(response_msg)
            
            # 重置导航状态
            self.is_navigating = False

    def add_to_conversation_history(self, text, role='assistant'):
        """将消息添加到LLM对话历史中"""
        message = {
            "role": role, 
            "content": text
        }
        history_msg = String()
        history_msg.data = json.dumps(message)
        
        # 发布到对话历史话题
        self.history_pub.publish(history_msg)
        self.get_logger().info(f'已添加消息到对话历史: {text}')

def main(args=None):
    rclpy.init(args=args)
    node = NavDialogueBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
