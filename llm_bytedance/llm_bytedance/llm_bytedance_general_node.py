#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import json
import os


class LLMBytedanceGeneralNode(Node):
    """
    ROS2节点，用于与火山引擎大模型API交互，实现一般场景的对话
    
    订阅话题:
        /speech_to_text: 接收语音识别的文本
    
    发布话题:
        /llm_response: 发布LLM生成的响应
        /llm_response_raw: 发布LLM生成的原始响应（不包含任何处理）
    """

    def __init__(self):
        super().__init__('llm_bytedance_general_node')
        
        # 获取API密钥，从环境变量获取
        self.api_key = os.environ.get('ARK_API_KEY', '')
        if not self.api_key:
            self.get_logger().error('未设置ARK_API_KEY环境变量，请先设置API密钥')
        
        # 火山引擎API配置
        self.api_url = "https://ark.cn-beijing.volces.com/api/v3/chat/completions"
        self.model_id = os.environ.get('ARK_MODEL_ID', 'ep-20250328110625-qxn6r')
        
        # 系统提示词，可以根据需要修改
        self.system_prompt = """【角色设定】
                    你是一个友好、乐于助人的校园导览机器人，名叫"瓦力/Wall-E"。你的主要任务是回答访客关于校园的问题，提供校园信息，并协助访客解决问题。

                    【行为准则】
                    1. 保持友好、礼貌的语气，使用简洁明了的语言
                    2. 回答应当简短精确，避免冗长解释
                    3. 当不确定答案时，诚实承认并提供可能的解决方案
                    4. 语言匹配原则：始终使用与用户最近一条消息相同的语言回答。如果用户使用中文，你应该用中文回答；如果用户使用英文，你应该用英文回答。
                    5. 不要假装你有实体形态或能够执行物理动作

                    【回答格式】
                    - 保持回答简洁，通常不超过3句话
                    - 使用自然、对话式的语言
                    - 不要使用markdown或其他格式标记
                    
                    【示例对话】
                    用户: "图书馆在哪里？"
                    回答: "图书馆位于校园中心区域，从这里向东走约5分钟就能到达。它是一栋白色砖墙的四层建筑。"
                    
                    用户: "Where is the library?"
                    回答: "The library is located in the central area of the campus, about a 5-minute walk eastward from here. It's a four-story building with white brick walls."
                    
                    用户: "你能帮我拿一本书吗？"
                    回答: "抱歉，我无法帮你拿书，因为我是一个虚拟助手。不过我可以告诉你图书馆的开放时间和借书流程，或者帮你联系图书馆工作人员获取帮助。"
        """
        
        # 对话历史
        self.conversation_history = []
        self.max_history_length = 10  # 最大保存的对话轮数
        self.initial_message_received = False  # 标记是否收到了初始消息
        
        # 创建订阅者和发布者
        self.subscription = self.create_subscription(
            String,
            '/speech_to_text',
            self.speech_callback,
            10
        )
        
        # 创建对话历史订阅者
        self.history_subscription = self.create_subscription(
            String,
            '/llm_conversation_history',
            self.conversation_history_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            String,
            '/llm_response',
            10
        )
        self.raw_publisher = self.create_publisher(
            String,
            '/llm_response_raw',
            10
        )
        
        self.get_logger().info('LLM Bytedance通用对话节点已启动')
    
    def conversation_history_callback(self, msg):
        """处理接收到的对话历史消息"""
        try:
            # 解析JSON消息
            history_message = json.loads(msg.data)
            
            if 'role' in history_message and 'content' in history_message:
                role = history_message['role']
                content = history_message['content']
                
                self.get_logger().info(f'添加对话历史: {role}: {content}')
                
                # 将消息添加到对话历史
                self.conversation_history.append({"role": role, "content": content})
                
                # 如果是助手角色的消息，标记已收到初始消息
                if role == 'assistant':
                    self.initial_message_received = True
                
                # 控制对话历史长度
                if len(self.conversation_history) > self.max_history_length * 2:
                    self.conversation_history = self.conversation_history[-self.max_history_length * 2:]
            else:
                self.get_logger().warn(f'收到格式不正确的对话历史消息: {msg.data}')
        
        except json.JSONDecodeError:
            self.get_logger().error(f'解析对话历史JSON消息失败: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'处理对话历史消息时出错: {str(e)}')
    
    def speech_callback(self, msg):
        """处理接收到的语音转文本消息"""
        text = msg.data
        self.get_logger().info(f'收到语音文本: {text}')
        
        if not text:
            self.get_logger().warn('收到空文本，跳过LLM请求')
            return
        
        # 将用户输入添加到对话历史
        self.conversation_history.append({"role": "user", "content": text})
        
        # 调用LLM API处理文本
        response_text = self.call_llm_api(text)
        
        if response_text:
            # 将LLM响应添加到对话历史
            self.conversation_history.append({"role": "assistant", "content": response_text})
            
            # 控制对话历史长度
            if len(self.conversation_history) > self.max_history_length * 2:  # 乘以2是因为每轮对话包含用户和助手各一条消息
                self.conversation_history = self.conversation_history[-self.max_history_length * 2:]
            
            # 将LLM响应发布到话题
            response_msg = String()
            response_msg.data = response_text
            self.publisher.publish(response_msg)
            self.raw_publisher.publish(response_msg)  # 同时发布原始响应
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
        
        # 准备消息列表，包含系统提示和对话历史
        messages = [{"role": "system", "content": self.system_prompt}]
        
        # 添加对话历史
        if self.conversation_history:
            # 只添加最近的几轮对话，避免超出token限制
            messages.extend(self.conversation_history[-self.max_history_length * 2:])
        else:
            # 如果没有对话历史，只添加当前用户输入
            messages.append({"role": "user", "content": user_text})
        
        # 准备请求体
        payload = {
            "model": self.model_id,
            "messages": messages
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
    node = LLMBytedanceGeneralNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
