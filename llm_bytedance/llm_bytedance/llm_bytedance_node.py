#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import json
import os


class LLMBytedanceNode(Node):
    """
    ROS2节点，用于与火山引擎大模型API交互
    
    订阅话题:
        /speech_to_text: 接收语音识别的文本
    
    发布话题:
        /llm_response: 发布LLM生成的响应
    """

    def __init__(self):
        super().__init__('llm_bytedance_node')
        
        # 获取API密钥，最好从环境变量获取
        self.api_key = os.environ.get('ARK_API_KEY', '')
        if not self.api_key:
            self.get_logger().error('未设置ARK_API_KEY环境变量，请先设置API密钥')
        
        # 火山引擎API配置
        self.api_url = "https://ark.cn-beijing.volces.com/api/v3/chat/completions"
        self.model_id = os.environ.get('ARK_MODEL_ID', 'ep-20250213183336-ngs4d')
        
        # 系统提示词，可以根据需要修改
        self.system_prompt = "你是一个智能助手，可以回答用户的问题并提供帮助，请你只输出尽量简短的文本，不要有任何表情符号的输出。"
        
        # 创建订阅者和发布者
        self.subscription = self.create_subscription(
            String,
            '/speech_to_text',
            self.speech_callback,
            10
        )
        self.publisher = self.create_publisher(
            String,
            '/llm_response',
            10
        )
        
        self.get_logger().info('LLM Bytedance节点已启动')
    
    def speech_callback(self, msg):
        """处理接收到的语音转文本消息"""
        text = msg.data
        self.get_logger().info(f'收到语音文本: {text}')
        
        if not text:
            self.get_logger().warn('收到空文本，跳过LLM请求')
            return
        
        # 调用LLM API处理文本
        response_text = self.call_llm_api(text)
        
        if response_text:
            # 将LLM响应发布到话题
            response_msg = String()
            response_msg.data = response_text
            self.publisher.publish(response_msg)
            self.get_logger().info(f'已发布LLM响应: {response_text}')
    
    def call_llm_api(self, user_text):
        """
        调用火山引擎大模型API
        
        Args:
            user_text: 用户输入的文本
            
        Returns:
            str: LLM生成的响应文本，如果出错则返回None
        """
        if not self.api_key:
            self.get_logger().error('API密钥未设置，无法调用LLM API')
            return None
        
        # 准备请求头部
        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.api_key}"
        }
        
        # 准备请求体
        payload = {
            "model": self.model_id,
            "messages": [
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": user_text}
            ]
        }
        
        try:
            # 发送请求
            self.get_logger().info('正在发送请求到火山引擎大模型API...')
            response = requests.post(self.api_url, headers=headers, json=payload)
            
            # 检查响应
            if response.status_code == 200:
                result = response.json()
                # 根据API返回格式提取文本
                if 'choices' in result and len(result['choices']) > 0:
                    if 'message' in result['choices'][0] and 'content' in result['choices'][0]['message']:
                        response_text = result['choices'][0]['message']['content']
                        self.get_logger().info(f'已收到LLM响应')
                        return response_text
                
                self.get_logger().error(f'无法从API响应中提取文本: {result}')
                return None
            else:
                self.get_logger().error(f'API请求失败: {response.status_code}, {response.text}')
                return None
        
        except Exception as e:
            self.get_logger().error(f'调用API时出错: {str(e)}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = LLMBytedanceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
