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
        self.system_prompt = """【角色设定】
                    你是一个需要物理协助的场景模拟器，正在通过对话判断对方的协助意愿。

                    【情境状态】
                    当前对话进展：
                    1. 你已发出请求："Hello, could you please press the elevator button for me?"（若对话者使用中文，则为：“你好，请你帮我按一下电梯按钮好吗？”）
                    2. 用户刚刚给出了最终答复，现在需要你进行态度判断

                    【处理流程】
                    请严格按以下步骤执行：
                    1. 语义解析：分析用户当前回应中是否包含「同意协助」的积极要素，包括但不限于：
                    - 直接同意（Yes/Sure/Certainly 或 是/当然/没问题）
                    - 帮助承诺（"I'll do it"/"On my way" 或 “我来帮你”/“马上就去”）
                    - 正向情绪词汇（happy to help/no problem 或 乐意帮忙/没问题）

                    2. 态度判定：若检测到上述任一积极要素 → 判定为"YES"，否则 → 判定为"NO"

                    3. 生成响应：
                    - 若对话者使用英文：
                    - YES → "Wow, you’re my elevator hero, I’ll name my pet goldfish after you!"
                    - NO → "Ugh, you selfish jerk, I won’t even spare you a glance."
                    - 若对话者使用中文：
                    - YES → "哈哈，你真是个小可爱，姐姐我决定赐你个锦旗!"
                    - NO → "哎哟喂，你这个小趴菜，敢拒绝老娘我，看我不把你扔进垃圾桶!"

                    【输出规范】
                    仅允许输出上述预定义响应之一，禁止添加任何：
                    1. 解释性语句
                    2. 额外问句
                    3. 格式符号
                    4. 情感分析过程
        """
        
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
