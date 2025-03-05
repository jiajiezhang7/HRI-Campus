#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2节点，使用阶跃星辰语音合成API将文本转换为语音
订阅LLM响应文本并发布合成的音频数据
"""

import os
import sys
import json
import time
import requests
import tempfile

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from audio_common_msgs.msg import AudioData

class SpeechGenerationStepfunNode(Node):
    """
    使用阶跃星辰语音合成API进行语音合成的ROS2节点
    """
    def __init__(self):
        super().__init__('speech_generation_stepfun_node')
        
        # 声明参数
        self.declare_parameter('voice', 'cixingnansheng')  # 音色选择，默认为磁性男声
        self.declare_parameter('response_format', 'mp3')  # 文件格式，支持 wav,mp3,flac,opus
        self.declare_parameter('speed', 1.0)  # 语速，范围0.5~2，默认1.0
        self.declare_parameter('volume', 1.0)  # 音量，范围0.1~2.0，默认1.0
        
        # 阶跃星辰API参数
        self.declare_parameter('stepfun_api_key', '')  # 阶跃星辰API密钥
        
        # 网络代理参数
        self.declare_parameter('use_proxy', False)  # 是否使用代理
        self.declare_parameter('http_proxy', '')    # HTTP代理
        self.declare_parameter('https_proxy', '')   # HTTPS代理
        
        # 获取参数
        self.voice = self.get_parameter('voice').value
        self.response_format = self.get_parameter('response_format').value
        self.speed = self.get_parameter('speed').value
        self.volume = self.get_parameter('volume').value
        
        # 获取阶跃星辰API参数
        self.api_key = self.get_parameter('stepfun_api_key').value
        
        # 获取网络代理参数
        self.use_proxy = self.get_parameter('use_proxy').value
        self.http_proxy = self.get_parameter('http_proxy').value
        self.https_proxy = self.get_parameter('https_proxy').value
        
        # 检查阶跃星辰API参数是否设置
        if not self.api_key:
            self.get_logger().error("阶跃星辰API参数未设置。请设置stepfun_api_key参数。")
        
        # 创建订阅者，订阅LLM响应文本
        self.text_subscription = self.create_subscription(
            String,
            '/llm_response',
            self.text_callback,
            10
        )
        
        # 创建发布者，发布合成的音频数据
        self.audio_publisher = self.create_publisher(
            AudioData,
            '/audio_generated',
            10
        )
        
        # 创建发布者，发布麦克风静音信号
        self.mute_publisher = self.create_publisher(
            Bool,
            '/mic_mute',
            10
        )
        
        # 创建调试文件目录
        self.debug_dir = os.path.expanduser('~/speech_generation_stepfun_debug')
        os.makedirs(self.debug_dir, exist_ok=True)
        
        # 设置阶跃星辰语音合成API的URL
        self.tts_url = 'https://api.stepfun.com/v1/audio/speech'
        
        # 配置代理
        self.proxies = None
        if self.use_proxy and (self.http_proxy or self.https_proxy):
            self.proxies = {}
            if self.http_proxy:
                self.proxies['http'] = self.http_proxy
                os.environ['http_proxy'] = self.http_proxy
            if self.https_proxy:
                self.proxies['https'] = self.https_proxy
                os.environ['https_proxy'] = self.https_proxy
        
        self.get_logger().info(f'语音合成节点已初始化')
        self.get_logger().info(f'使用阶跃星辰语音合成API')
        self.get_logger().info(f'音色: {self.voice}, 语速: {self.speed}, 音量: {self.volume}')
        self.get_logger().info(f'输出格式: {self.response_format}')
    
    def text_to_speech(self, text):
        """
        将文本转换为语音
        """
        self.get_logger().info(f"开始将文本转换为语音: {text[:30]}...")
        
        if not self.api_key:
            self.get_logger().error("阶跃星辰API密钥未设置，无法进行语音合成")
            return None
        
        try:
            # 创建请求头
            headers = {
                'Authorization': f'Bearer {self.api_key}'
            }
            
            # 创建请求数据
            data = {
                'model': 'step-tts-mini',
                'input': text,
                'voice': self.voice,
                'response_format': self.response_format,
                'speed': self.speed,
                'volume': self.volume
            }
            
            self.get_logger().debug(f"请求阶跃星辰语音合成API: {self.tts_url}")
            
            # 发送请求
            response = requests.post(
                self.tts_url,
                headers=headers,
                json=data,
                proxies=self.proxies
            )
            
            # 检查响应
            if response.status_code == 200:
                # 获取音频数据
                audio_data = response.content
                self.get_logger().info(f"语音合成成功，获取到{len(audio_data)}字节的音频数据")
                
                # 保存音频文件用于调试
                timestamp = time.strftime("%Y%m%d-%H%M%S")
                audio_file = os.path.join(self.debug_dir, f'audio_{timestamp}.{self.response_format}')
                with open(audio_file, 'wb') as of:
                    of.write(audio_data)
                
                self.get_logger().info(f"已保存音频文件用于调试: {audio_file}")
                return audio_data
            else:
                # 保存错误信息用于调试
                timestamp = time.strftime("%Y%m%d-%H%M%S")
                error_file = os.path.join(self.debug_dir, f'error_{timestamp}.txt')
                with open(error_file, 'wb') as of:
                    of.write(response.content)
                
                self.get_logger().error(f"语音合成API请求失败，状态码: {response.status_code}")
                try:
                    error_json = response.json()
                    self.get_logger().error(f"错误详情: {json.dumps(error_json, ensure_ascii=False)}")
                except:
                    self.get_logger().error(f"无法解析错误响应，详情请查看: {error_file}")
                
                return None
        
        except Exception as e:
            self.get_logger().error(f"语音合成过程中发生错误: {str(e)}")
            return None
    
    def text_callback(self, msg):
        """
        处理接收到的文本消息，转换为语音
        """
        if not msg.data:
            self.get_logger().warn("收到空文本消息，忽略")
            return
        
        self.get_logger().info(f"收到文本消息: {msg.data[:50]}...")
        
        # 在开始语音合成前，先发送麦克风静音信号
        mute_msg = Bool()
        mute_msg.data = True
        self.mute_publisher.publish(mute_msg)
        self.get_logger().info("已发送麦克风静音信号")
        
        # 将文本转换为语音
        audio_data = self.text_to_speech(msg.data)
        
        if audio_data:
            # 发布音频数据
            audio_msg = AudioData()
            audio_msg.data = audio_data
            self.audio_publisher.publish(audio_msg)
            self.get_logger().info("已发布合成的音频数据")
        else:
            self.get_logger().error("语音合成失败，无法发布音频数据")


def main(args=None):
    rclpy.init(args=args)
    
    node = SpeechGenerationStepfunNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
