#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
通用场景主动发问节点，发送预设问题到语音合成模块
提供服务接口供其他节点调用
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Empty, SetBool
import time
import random
import json


class ActiveQuestioningGeneralNode(Node):
    """
    通用场景主动发问节点，发送预设问题到语音合成模块
    提供服务接口供其他节点调用
    支持多种场景下的问题模板
    """
    def __init__(self):
        super().__init__('active_questioning_general_node')
        
        # 创建发布者，发布到LLM响应话题
        self.text_publisher = self.create_publisher(
            String,
            '/llm_response',
            10
        )
        
        # 创建发布者，发布到LLM对话历史话题
        self.history_publisher = self.create_publisher(
            String,
            '/llm_conversation_history',
            10
        )
        
        # 创建可靠的QoS配置
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # 可靠传输
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # 持久性，新订阅者可以收到之前发布的消息
            history=QoSHistoryPolicy.KEEP_LAST,  # 保留最后N条消息
            depth=10  # 队列大小
        )
        
        # 场景类型和对应的问题模板（英语）
        self.scene_templates = {
            'greeting': [
                "Hello, I'm Wall-E, the campus guide robot. How can I help you today?",
                "Hi, I'm Wall-E. Nice to meet you! Would you like me to introduce the campus environment?",
                "Welcome to our campus! I'm the guide robot Wall-E. Is there anything I can assist you with?"
            ],
            'direction': [
                "Do you know how to get to the library?",
                "I'd like to go to the student center. Could you tell me the way?",
                "Which direction is the cafeteria?"
            ],
            'information': [
                "Do you know what activities are happening on campus today?",
                "What are the opening hours of the library?",
                "What are some places worth visiting on campus?"
            ],
            'help': [
                "Could you help me with directions?",
                "I think I'm lost. Can you help me find my way?",
                "Could you tell me where the nearest restroom is?"
            ]
        }
        
        # 当前场景类型
        self.current_scene = 'greeting'
        
        # 当前问题文本
        self.question_text = self.get_random_question('greeting')
        
        # 声明参数
        self.declare_parameter('delay', 1.0)  # 延迟发问的时间（秒）
        self.declare_parameter('check_subscribers', True)  # 是否检查订阅者
        self.declare_parameter('max_retries', 10)  # 最大重试次数
        self.declare_parameter('retry_interval', 1.0)  # 重试间隔（秒）
        self.declare_parameter('default_scene', 'greeting')  # 默认场景类型
        
        # 获取参数
        self.delay = self.get_parameter('delay').value
        self.check_subscribers = self.get_parameter('check_subscribers').value
        self.max_retries = self.get_parameter('max_retries').value
        self.retry_interval = self.get_parameter('retry_interval').value
        self.current_scene = self.get_parameter('default_scene').value
        
        # 创建服务，允许外部触发发问
        self.trigger_service = self.create_service(
            Empty,
            '/active_questioning/trigger_question',
            self.trigger_callback
        )
        
        # 创建服务，允许设置场景类型
        self.set_scene_service = self.create_service(
            SetBool,
            '/active_questioning/set_scene',
            self.set_scene_callback
        )
        
        # 创建服务，允许设置自定义问题
        self.set_question_service = self.create_service(
            String,
            '/active_questioning/set_question',
            self.set_question_callback
        )
        
        self.get_logger().info('通用场景主动发问节点已初始化，等待服务调用')
    
    def get_random_question(self, scene_type):
        """
        从指定场景类型中随机获取一个问题
        """
        if scene_type in self.scene_templates and self.scene_templates[scene_type]:
            return random.choice(self.scene_templates[scene_type])
        else:
            self.get_logger().warn(f'未找到场景类型 "{scene_type}" 的问题模板，使用默认问候语')
            return "Hello, I'm Xiao Zhi, the campus guide robot. How can I help you today?"
    
    def ask_question(self):
        """
        发送问题到语音合成模块
        """
        # 检查是否有订阅者
        if self.check_subscribers and self.text_publisher.get_subscription_count() == 0:
            self.get_logger().warn('没有检测到订阅者，将尝试重试发送消息')
            
            # 创建重试定时器
            self.retry_count = 0
            self.retry_timer = self.create_timer(self.retry_interval, self.retry_ask_question)
            return
        
        # 记录使用的问题文本
        self.get_logger().info(f'发送问题: "{self.question_text}"')
        
        # 创建消息并发布到语音合成话题
        msg = String()
        msg.data = self.question_text
        self.text_publisher.publish(msg)
        
        # 将主动发问的内容添加到LLM的对话历史中
        self.add_to_conversation_history(self.question_text, 'assistant')
    
    def retry_ask_question(self):
        """
        重试发送问题
        """
        self.retry_count += 1
        
        # 检查是否有订阅者
        if self.text_publisher.get_subscription_count() > 0:
            # 记录使用的问题文本
            self.get_logger().info(f'发送问题: "{self.question_text}"')
            
            # 创建消息并发布到语音合成话题
            msg = String()
            msg.data = self.question_text
            self.text_publisher.publish(msg)
            
            # 将主动发问的内容添加到LLM的对话历史中
            self.add_to_conversation_history(self.question_text, 'assistant')
            
            # 取消重试定时器
            self.retry_timer.cancel()
            delattr(self, 'retry_timer')
            return
        
        # 检查是否达到最大重试次数
        if self.retry_count >= self.max_retries:
            self.get_logger().error(f'达到最大重试次数({self.max_retries})，放弃发送消息')
            
            # 取消重试定时器
            self.retry_timer.cancel()
            delattr(self, 'retry_timer')
            return
        
        self.get_logger().warn(f'重试 {self.retry_count}/{self.max_retries}: 没有检测到订阅者')
    
    def trigger_callback(self, request, response):
        """
        处理触发服务的回调
        """
        self.get_logger().info('收到触发主动发问的请求')
        
        # 更新问题文本（随机选择当前场景类型的问题）
        self.question_text = self.get_random_question(self.current_scene)
        
        # 添加延迟
        if self.delay > 0:
            self.get_logger().debug(f'延迟 {self.delay} 秒后发送问题')
            time.sleep(self.delay)
        
        self.ask_question()
        return response
    
    def set_scene_callback(self, request, response):
        """
        处理设置场景类型的回调
        
        请求中的data字段应为场景类型名称
        """
        scene_type = request.data
        
        if scene_type in self.scene_templates:
            self.current_scene = scene_type
            self.get_logger().info(f'已设置场景类型为: {scene_type}')
            
            # 更新问题文本
            self.question_text = self.get_random_question(self.current_scene)
            
            response.success = True
            response.message = f"Successfully set scene type to: {scene_type}"
        else:
            self.get_logger().warn(f'未知的场景类型: {scene_type}')
            response.success = False
            response.message = f"Unknown scene type: {scene_type}, valid scene types: {list(self.scene_templates.keys())}"
        
        return response
    
    def set_question_callback(self, request, response):
        """
        处理设置自定义问题的回调
        """
        custom_question = request.data
        
        if custom_question:
            self.question_text = custom_question
            self.get_logger().info(f'已设置自定义问题: {custom_question}')
            response.data = "Successfully set custom question"
        else:
            self.get_logger().warn('收到空的自定义问题，忽略')
            response.data = "Custom question cannot be empty"
        
        return response


    def add_to_conversation_history(self, text, role):
        """
        将消息添加到LLM的对话历史中
        
        Args:
            text: 消息文本
            role: 消息角色，'assistant'或'user'
        """
        # 创建对话历史消息
        history_message = {
            "role": role,
            "content": text
        }
        
        # 将消息转换为JSON字符串
        history_json = json.dumps(history_message)
        
        # 创建消息并发布到对话历史话题
        msg = String()
        msg.data = history_json
        self.history_publisher.publish(msg)
        self.get_logger().debug(f'已添加到对话历史: {role}: {text}')


def main(args=None):
    rclpy.init(args=args)
    
    node = ActiveQuestioningGeneralNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
