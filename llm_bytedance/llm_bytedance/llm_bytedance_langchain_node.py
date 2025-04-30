#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
import yaml
from ament_index_python.packages import get_package_share_directory
import requests
from typing import Any, List, Optional

# LangChain 导入
from langchain.llms.base import LLM
from langchain.chains import LLMChain
from langchain.prompts import PromptTemplate
from langchain.memory import ConversationBufferMemory
from langchain.callbacks.manager import CallbackManagerForLLMRun


class VolcanoEngineLLM(LLM):
    """火山引擎大模型的LangChain接口"""
    
    api_key: str = ""
    api_url: str = "https://ark.cn-beijing.volces.com/api/v3/chat/completions"
    model_id: str = "ep-20250328110625-qxn6r"
    system_prompt: str = "You are a helpful assistant."
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # 获取API密钥，从环境变量获取
        self.api_key = os.environ.get('ARK_API_KEY', '8967f487-1a84-4340-8ac5-79f087456b95')
        self.model_id = os.environ.get('ARK_MODEL_ID', 'ep-20250328110625-qxn6r')
        
    @property
    def _llm_type(self) -> str:
        """返回LLM类型"""
        return "volcano-engine"
    
    def _call(
        self,
        prompt: str,
        stop: Optional[List[str]] = None,
        run_manager: Optional[CallbackManagerForLLMRun] = None,
        **kwargs: Any,
    ) -> str:
        """调用火山引擎大模型API"""
        if not self.api_key:
            raise ValueError("API密钥未设置，无法调用LLM API")
        
        # 准备请求头部
        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.api_key}"
        }
        
        # 从LangChain的内存中获取对话历史
        # 这里假设我们已经通过LLMChain传入了对话历史
        # 准备消息列表，始终包含系统提示
        messages = [{"role": "system", "content": self.system_prompt}]
        
        # 添加当前用户输入
        messages.append({"role": "user", "content": prompt})
        
        # 准备请求体
        payload = {
            "model": self.model_id,
            "messages": messages
        }
        
        try:
            # 发送请求
            response = requests.post(self.api_url, headers=headers, json=payload)
            
            # 检查响应
            if response.status_code == 200:
                result = response.json()
                # 根据API返回格式提取文本
                if 'choices' in result and len(result['choices']) > 0:
                    if 'message' in result['choices'][0] and 'content' in result['choices'][0]['message']:
                        response_text = result['choices'][0]['message']['content']
                        return response_text
                
                error_msg = f'无法从API响应中提取文本: {result}'
                if run_manager:
                    run_manager.on_text(error_msg)
                return "API响应解析错误"
            else:
                error_msg = f'API请求失败: {response.status_code}, {response.text}'
                if run_manager:
                    run_manager.on_text(error_msg)
                return f"API请求失败: {response.status_code}"
        
        except Exception as e:
            error_msg = f'调用API时出错: {str(e)}'
            if run_manager:
                run_manager.on_text(error_msg)
            return f"API调用错误: {str(e)}"


class LLMBytedanceLangchainNode(Node):
    """
    ROS2节点，使用LangChain与火山引擎大模型API交互，实现一般场景的对话
    
    订阅话题:
        /speech_to_text: 接收语音识别的文本
        /llm_conversation_history: 接收对话历史消息
    
    发布话题:
        /llm_response: 发布LLM生成的响应
        /llm_response_raw: 发布LLM生成的原始响应（不包含任何处理）
    """

    def __init__(self):
        super().__init__('llm_bytedance_langchain_node')
        
        # Declare and get the prompt_type parameter
        self.declare_parameter('prompt_type', 'general')
        self.prompt_type = self.get_parameter('prompt_type').get_parameter_value().string_value
        self.get_logger().info(f'Using prompt type: {self.prompt_type}')

        # 从YAML文件加载系统提示词，使用参数指定的类型
        self.system_prompt = self._load_system_prompt_from_yaml(self.prompt_type)
        if not self.system_prompt:
             # Handle case where prompt loading fails, maybe use a default or raise error
             self.get_logger().error(f'Failed to load system prompt for type "{self.prompt_type}" from YAML. Using a default prompt.')
             # 设置一个最小化的默认提示词，以防加载失败
             self.system_prompt = "You are a helpful assistant named Wall-E."

        # 创建自定义LLM
        self.llm = VolcanoEngineLLM()
        self.llm.system_prompt = self.system_prompt

        # 创建提示模板
        self.template = """
        {history}
        Human: {human_input}
        AI:"""
        
        self.prompt = PromptTemplate(
            input_variables=["history", "human_input"],
            template=self.template
        )
        
        # 创建对话记忆
        self.memory = ConversationBufferMemory(memory_key="history")
        
        # 创建LLM链
        self.chain = LLMChain(
            llm=self.llm,
            prompt=self.prompt,
            memory=self.memory,
            verbose=False
        )

        # 设置对话历史长度限制
        self.max_history_length = 10  # 最大保存的对话轮数
        
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
        self.raw_publisher = self.create_publisher(
            String,
            '/llm_response_raw',
            10
        )
        
        self.get_logger().info('LLM Bytedance Langchain节点已启动')
    
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


    
    def speech_callback(self, msg):
        """处理接收到的语音转文本消息"""
        text = msg.data
        self.get_logger().info(f'收到语音文本: {text}')
        
        if not text:
            self.get_logger().warn('收到空文本，跳过LLM请求')
            return
        
        # 使用LangChain调用LLM
        self.get_logger().info('正在发送请求到火山引擎大模型API...')
        try:
            # 使用LangChain链进行调用，LangChain会自动管理对话历史
            response_text = self.chain.run(
                human_input=text
            )
            
            self.get_logger().info('已收到LLM响应')
            
            if response_text:
                # 将LLM响应发布到话题
                response_msg = String()
                response_msg.data = response_text
                self.publisher.publish(response_msg)
                self.raw_publisher.publish(response_msg)  # 同时发布原始响应
                self.get_logger().info(f'已发布LLM响应: {response_text}')
        
        except Exception as e:
            self.get_logger().error(f'调用LLM时出错: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = LLMBytedanceLangchainNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
