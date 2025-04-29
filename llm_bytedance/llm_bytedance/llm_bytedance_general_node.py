#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import json
import os
import yaml
from ament_index_python.packages import get_package_share_directory


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
        
        # Declare and get the prompt_type parameter
        self.declare_parameter('prompt_type', 'general')
        self.prompt_type = self.get_parameter('prompt_type').get_parameter_value().string_value
        self.get_logger().info(f'Using prompt type: {self.prompt_type}')

        # 获取API密钥，从环境变量获取
        self.api_key = os.environ.get('ARK_API_KEY', '')
        if not self.api_key:
            self.get_logger().error('未设置ARK_API_KEY环境变量，请先设置API密钥')
        
        # 火山引擎API配置
        self.api_url = "https://ark.cn-beijing.volces.com/api/v3/chat/completions"
        self.model_id = os.environ.get('ARK_MODEL_ID', 'ep-20250328110625-qxn6r')
        
        # 从YAML文件加载系统提示词，使用参数指定的类型
        self.system_prompt = self._load_system_prompt_from_yaml(self.prompt_type)
        if not self.system_prompt:
             # Handle case where prompt loading fails, maybe use a default or raise error
             self.get_logger().error(f'Failed to load system prompt for type "{self.prompt_type}" from YAML. Using a default prompt.')
             # 设置一个最小化的默认提示词，以防加载失败
             self.system_prompt = "You are a helpful assistant named Wall-E."


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
    
    def _load_system_prompt_from_yaml(self, prompt_type):
        """从YAML文件中加载指定类型的系统提示词"""
        package_share_directory = None
        yaml_file_path = None
        try:
            # 获取包的共享目录路径
            package_share_directory = get_package_share_directory('llm_bytedance')
            # 构建YAML文件的完整路径
            yaml_file_path = os.path.join(package_share_directory, 'config', 'system_prompts.yaml')
            
            self.get_logger().info(f'Loading system prompt from: {yaml_file_path}')

            # 检查文件是否存在
            if not os.path.exists(yaml_file_path):
                self.get_logger().error(f'System prompt file not found at: {yaml_file_path}')
                return None

            # 读取并解析YAML文件
            with open(yaml_file_path, 'r', encoding='utf-8') as file:
                prompts = yaml.safe_load(file)
                # 检查指定的 prompt_type 和 'prompt' 是否存在
                if prompts and prompt_type in prompts and 'prompt' in prompts[prompt_type]:
                    self.get_logger().info(f'Successfully loaded "{prompt_type}" system prompt from YAML.')
                    return prompts[prompt_type]['prompt']
                else:
                    self.get_logger().error(f'Could not find "{prompt_type}.prompt" key structure in the YAML file.')
                    return None
        except FileNotFoundError:
            # 这个错误理论上会被 os.path.exists 捕获，但保留以防万一
            self.get_logger().error(f'System prompt file not found (FileNotFoundError). Path: {yaml_file_path}')
            return None
        except yaml.YAMLError as e:
            self.get_logger().error(f'Error parsing system prompt YAML file: {e}')
            return None
        except Exception as e:
            # 捕获其他潜在错误，例如权限问题
            self.get_logger().error(f'An unexpected error occurred while loading system prompt: {e}')
            return None

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
