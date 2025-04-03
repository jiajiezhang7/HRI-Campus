#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
主动发问节点，发送固定文本到语音合成模块
提供服务接口供其他节点调用
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Empty
from level_interfaces.msg import Level
import time

class ActiveQuestioningNode(Node):
    """
    主动发问节点，发送固定问题到语音合成模块
    提供服务接口供其他节点调用
    根据电梯楼层信息调整问题内容
    """
    def __init__(self):
        super().__init__('active_questioning_node')
        
        # 创建发布者，发布到LLM响应话题
        self.text_publisher = self.create_publisher(
            String,
            '/llm_response',
            10
        )
        
        # 标记是否已收到电梯信息
        self.received_level_info = False
        self.last_level_received_time = None
        
        # 创建可靠的QoS配置
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # 可靠传输
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # 持久性，新订阅者可以收到之前发布的消息
            history=QoSHistoryPolicy.KEEP_LAST,  # 保留最后N条消息
            depth=10  # 队列大小
        )
        
        # 创建订阅者，订阅电梯楼层信息，使用可靠的QoS配置
        self.level_subscriber = self.create_subscription(
            Level,
            '/dummy_level',
            self.level_callback,
            qos_profile=reliable_qos
        )
        
        # 当前电梯信息
        self.current_level_info = None
        
        # 基础问题文本模板
        self.up_template = "Hello, could you please press the elevator's up button for me? I'd like to go to the %d floor."
        self.down_template = "Hello, could you please press the elevator's down button for me? I'd like to go to the %d floor."
        
        # 默认问题文本（兼容没有接收到电梯信息的情况）
        self.question_text = "Hello, could you please press the elevator button for me?"
        
        # 声明参数
        self.declare_parameter('delay', 1.0)  # 延迟发问的时间（秒）
        self.declare_parameter('check_subscribers', True)  # 是否检查订阅者
        self.declare_parameter('max_retries', 10)  # 最大重试次数
        self.declare_parameter('retry_interval', 1.0)  # 重试间隔（秒）
        
        # 获取参数
        self.delay = self.get_parameter('delay').value
        self.check_subscribers = self.get_parameter('check_subscribers').value
        self.max_retries = self.get_parameter('max_retries').value
        self.retry_interval = self.get_parameter('retry_interval').value
        
        # 创建服务，允许外部触发发问
        self.trigger_service = self.create_service(
            Empty,
            '/active_questioning/trigger_question',
            self.trigger_callback
        )
        
        self.get_logger().info('主动发问节点已初始化，等待服务调用')
        
        # 创建定时器检查电梯信息
        self.create_timer(5.0, self.check_level_info)
    
    def check_level_info(self):
        """
        定期检查电梯信息的可用性
        """
        if not self.received_level_info:
            self.get_logger().warn('尚未接收到任何电梯信息，将使用默认问题文本')
        else:
            self.get_logger().debug(f'最后一次接收到电梯信息的时间: {self.last_level_received_time}')
    
    def level_callback(self, msg):
        """
        处理接收到的电梯楼层信息
        """
        self.current_level_info = msg
        self.received_level_info = True
        self.last_level_received_time = self.get_clock().now().to_msg()
        
        self.get_logger().debug(f'接收到电梯信息：方向={msg.is_up}, 楼层={msg.level}, 时间={self.last_level_received_time}')
        
        # 根据电梯信息更新问题文本
        if msg.is_up:
            self.question_text = self.up_template % msg.level
        else:
            self.question_text = self.down_template % msg.level
    
    def ask_question(self):
        """
        发送固定问题到语音合成模块
        """
        # 检查是否已收到电梯信息
        if not self.received_level_info:
            self.get_logger().error('未接收到电梯信息，无法发送问题')
            return
            
        # 检查是否有订阅者
        if self.check_subscribers and self.text_publisher.get_subscription_count() == 0:
            self.get_logger().warn('没有检测到订阅者，将尝试重试发送消息')
            
            # 创建重试定时器
            self.retry_count = 0
            self.retry_timer = self.create_timer(self.retry_interval, self.retry_ask_question)
            return
        
        # 记录使用的问题文本类型
        self.get_logger().info(f'使用基于电梯信息的问题文本: "{self.question_text}"')
        
        # 创建消息并发布
        msg = String()
        msg.data = self.question_text
        self.text_publisher.publish(msg)
    
    def retry_ask_question(self):
        """
        重试发送问题
        """
        self.retry_count += 1
        
        # 检查是否有订阅者
        if self.text_publisher.get_subscription_count() > 0:
            # 记录使用的问题文本类型
            self.get_logger().info(f'使用基于电梯信息的问题文本: "{self.question_text}"')
            
            # 创建消息并发布
            msg = String()
            msg.data = self.question_text
            self.text_publisher.publish(msg)
            
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
        
        # 检查是否已收到电梯信息
        if not self.received_level_info:
            self.get_logger().warn('尚未接收到电梯信息，等待接收...')
            # 创建等待电梯信息的定时器
            self.wait_count = 0
            self.max_wait_count = 20  # 最多等待20次
            self.wait_timer = self.create_timer(0.5, self.wait_for_level_info)  # 每0.5秒检查一次
            # 保存响应对象，以便在收到电梯信息后返回
            self.pending_response = response
            return response
        
        self.ask_question()
        return response
        
    def wait_for_level_info(self):
        """
        等待接收电梯信息
        """
        self.wait_count += 1
        
        # 检查是否已收到电梯信息
        if self.received_level_info:
            self.get_logger().info(f'已接收到电梯信息，发送问题')
            # 取消等待定时器
            self.wait_timer.cancel()
            delattr(self, 'wait_timer')
            # 发送问题
            self.ask_question()
            return
        
        # 检查是否达到最大等待次数
        if self.wait_count >= self.max_wait_count:
            self.get_logger().error(f'等待电梯信息超时({self.max_wait_count * 0.5}秒)，放弃发送问题')
            # 取消等待定时器
            self.wait_timer.cancel()
            delattr(self, 'wait_timer')
            return
        
        self.get_logger().warn(f'等待电梯信息 {self.wait_count}/{self.max_wait_count}...')


def main(args=None):
    rclpy.init(args=args)
    
    node = ActiveQuestioningNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
