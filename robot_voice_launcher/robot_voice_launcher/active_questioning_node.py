#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
主动发问节点，发送固定文本到语音合成模块
提供服务接口供其他节点调用
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty
import time

class ActiveQuestioningNode(Node):
    """
    主动发问节点，发送固定问题到语音合成模块
    提供服务接口供其他节点调用
    """
    def __init__(self):
        super().__init__('active_questioning_node')
        
        # 创建发布者，发布到LLM响应话题
        self.text_publisher = self.create_publisher(
            String,
            '/llm_response',
            10
        )
        
        # 固定的问题文本
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
    
    def ask_question(self):
        """
        发送固定问题到语音合成模块
        """
        # 检查是否有订阅者
        if self.check_subscribers and self.text_publisher.get_subscription_count() == 0:
            self.get_logger().warn('没有检测到订阅者，将尝试重试发送消息')
            
            # 创建重试定时器
            self.retry_count = 0
            self.retry_timer = self.create_timer(self.retry_interval, self.retry_ask_question)
            return
        
        self.get_logger().info(f'发送主动问题: "{self.question_text}"')
        
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
            self.get_logger().info(f'检测到订阅者，发送主动问题: "{self.question_text}"')
            
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
        self.ask_question()
        return response


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
