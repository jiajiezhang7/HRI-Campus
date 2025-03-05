#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2节点，使用阶跃星辰语音识别API进行语音识别
订阅音频数据并发布识别出的文本
"""

import os
import sys
import time
import wave
import json
import base64
import numpy as np
import requests
import tempfile
import threading
import struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData

class SpeechRecognitionStepfunNode(Node):
    """
    使用阶跃星辰语音识别API进行语音识别的ROS2节点
    """
    def __init__(self):
        super().__init__('speech_recognition_stepfun_node')
        
        # 声明参数
        self.declare_parameter('language', 'zh-cn')  # 默认使用中文
        self.declare_parameter('buffer_size', 32000)  # 音频缓冲区大小
        self.declare_parameter('silence_threshold', 200)  # 静音阈值
        self.declare_parameter('min_utterance_length', 16000)  # 最小语句长度（约0.5秒）
        self.declare_parameter('silence_duration', 1.5)  # 静音持续时间（秒）判定为句子结束
        self.declare_parameter('channels', 2)  # 声道数
        self.declare_parameter('sample_width', 2)  # 采样宽度（字节）
        self.declare_parameter('sample_rate', 16000)  # 采样率
        
        # 阶跃星辰API参数
        self.declare_parameter('stepfun_api_key', '')  # 阶跃星辰API密钥
        
        # 网络代理参数
        self.declare_parameter('use_proxy', False)  # 是否使用代理
        self.declare_parameter('http_proxy', '')    # HTTP代理
        self.declare_parameter('https_proxy', '')   # HTTPS代理
        
        # 获取参数
        self.language = self.get_parameter('language').value
        self.buffer_size = self.get_parameter('buffer_size').value
        self.silence_threshold = self.get_parameter('silence_threshold').value
        self.min_utterance_length = self.get_parameter('min_utterance_length').value
        self.silence_duration = self.get_parameter('silence_duration').value
        self.channels = self.get_parameter('channels').value
        self.sample_width = self.get_parameter('sample_width').value
        self.sample_rate = self.get_parameter('sample_rate').value
        
        # 获取阶跃星辰API参数
        self.api_key = self.get_parameter('stepfun_api_key').value
        
        # 获取网络代理参数
        self.use_proxy = self.get_parameter('use_proxy').value
        self.http_proxy = self.get_parameter('http_proxy').value
        self.https_proxy = self.get_parameter('https_proxy').value
        
        # 检查阶跃星辰API参数是否设置
        if not self.api_key:
            self.get_logger().error("阶跃星辰API参数未设置。请设置stepfun_api_key参数。")
        
        # 创建订阅者，订阅音频数据
        self.audio_subscription = self.create_subscription(
            AudioData,
            '/audio/audio',
            self.audio_callback,
            10
        )
        
        # 创建发布者，发布到/speech_to_text话题
        self.speech_to_text_publisher = self.create_publisher(
            String,
            '/speech_to_text',
            10
        )
        
        # 音频缓冲区，用于累积足够的音频数据进行识别
        self.audio_buffer = bytearray()
        
        # 语音活动检测相关变量
        self.is_speaking = False
        self.silence_start_time = None
        self.utterance_buffer = bytearray()
        
        # 创建调试文件目录
        self.debug_dir = os.path.expanduser('~/speech_recognition_stepfun_debug')
        os.makedirs(self.debug_dir, exist_ok=True)
        
        # 创建原始数据文件，用于调试
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        self.raw_file = open(os.path.join(self.debug_dir, f'raw_audio_{timestamp}.bin'), 'wb')
        self.get_logger().info(f'创建原始数据文件: {self.raw_file.name}')
        
        # 创建定时器，每10秒报告一次状态
        self.timer = self.create_timer(10.0, self.report_status)
        
        # 阶跃星辰语音识别API的URL
        self.asr_url = 'https://api.stepfun.com/v1/audio/transcriptions'
        
        self.get_logger().info(f'语音识别节点已初始化，使用语言: {self.language}')
        self.get_logger().info(f'音频格式: {self.channels}声道, {self.sample_width*8}位, {self.sample_rate}Hz')
        self.get_logger().info('使用阶跃星辰语音识别API')
    
    def is_silence(self, audio_data):
        """
        判断音频数据是否为静音
        """
        # 将字节数据转换为16位整数数组
        try:
            # 假设是16位音频数据
            audio_array = np.frombuffer(audio_data, dtype=np.int16)
            # 计算音频能量
            energy = np.sqrt(np.mean(np.square(audio_array.astype(np.float32))))
            return energy < self.silence_threshold
        except Exception as e:
            self.get_logger().error(f'音频数据处理错误: {str(e)}')
            return True
    
    def audio_callback(self, msg):
        """
        处理接收到的音频数据，使用VAD检测完整句子
        """
        # 保存原始数据用于调试
        if hasattr(self, 'raw_file') and self.raw_file:
            self.raw_file.write(bytes(msg.data))
            self.raw_file.flush()
        
        # 检查数据是否有效
        if len(msg.data) == 0:
            return
        
        try:
            # 打印一些样本值用于调试
            if len(msg.data) >= 20:
                samples = struct.unpack(f'<{min(10, len(msg.data)//2)}h', 
                                      msg.data[:min(20, len(msg.data))])
                self.get_logger().debug(f'样本值: {samples}')
            
            # 将音频数据添加到临时缓冲区
            self.audio_buffer.extend(msg.data)
            
            # 当缓冲区足够大时，进行VAD处理
            if len(self.audio_buffer) >= self.buffer_size:
                is_silence = self.is_silence(self.audio_buffer)
                
                # 如果当前不在说话状态，且检测到声音，开始新的语句
                if not self.is_speaking and not is_silence:
                    self.is_speaking = True
                    self.utterance_buffer = bytearray()
                    self.utterance_buffer.extend(self.audio_buffer)
                    self.get_logger().info('检测到语音开始')
                    
                # 如果当前在说话状态
                elif self.is_speaking:
                    # 添加当前音频到语句缓冲区
                    self.utterance_buffer.extend(self.audio_buffer)
                    
                    # 如果检测到静音，记录静音开始时间
                    if is_silence:
                        if self.silence_start_time is None:
                            self.silence_start_time = time.time()
                        
                        # 如果静音持续足够长时间，且语句长度足够，认为语句结束
                        if (time.time() - self.silence_start_time > self.silence_duration and 
                            len(self.utterance_buffer) > self.min_utterance_length):
                            self.get_logger().info('检测到语音结束，进行识别')
                            self.recognize_speech(self.utterance_buffer)
                            self.is_speaking = False
                            self.silence_start_time = None
                            self.utterance_buffer = bytearray()
                    else:
                        # 如果不是静音，重置静音开始时间
                        self.silence_start_time = None
                
                # 清空临时缓冲区
                self.audio_buffer = bytearray()
        except Exception as e:
            self.get_logger().error(f'处理音频数据错误: {str(e)}')
    
    def report_status(self):
        """
        定期报告节点状态
        """
        self.get_logger().info(f'语音识别节点状态: 正在运行')
        self.get_logger().info(f'当前是否在说话: {self.is_speaking}')
        if self.is_speaking:
            self.get_logger().info(f'当前语句缓冲区大小: {len(self.utterance_buffer)} 字节')
    
    def save_audio_to_file(self, audio_data, file_path):
        """
        将音频数据保存为WAV文件
        """
        try:
            with wave.open(file_path, 'wb') as wf:
                wf.setnchannels(self.channels)
                wf.setsampwidth(self.sample_width)
                wf.setframerate(self.sample_rate)
                wf.writeframes(audio_data)
            self.get_logger().info(f'音频数据已保存到: {file_path}')
            return True
        except Exception as e:
            self.get_logger().error(f'保存音频数据错误: {str(e)}')
            return False
    
    def recognize_speech(self, audio_data):
        """
        使用阶跃星辰API识别语音
        """
        if not self.api_key:
            self.get_logger().error("阶跃星辰API密钥未设置，无法进行语音识别")
            return
        
        # 设置代理（如果需要）
        proxies = None
        if self.use_proxy and (self.http_proxy or self.https_proxy):
            proxies = {}
            if self.http_proxy:
                proxies['http'] = self.http_proxy
            if self.https_proxy:
                proxies['https'] = self.https_proxy
        
        # 创建临时WAV文件
        try:
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
                temp_wav_path = temp_file.name
            
            # 保存音频数据到临时文件
            if not self.save_audio_to_file(audio_data, temp_wav_path):
                self.get_logger().error("无法保存音频数据到临时文件")
                return
            
            # 创建请求头和数据
            headers = {
                'Authorization': f'Bearer {self.api_key}'
            }
            
            # 准备文件和表单数据
            files = {
                'file': open(temp_wav_path, 'rb')
            }
            
            data = {
                'model': 'step-asr',
                'response_format': 'json'
            }
            
            # 发送请求到阶跃星辰API
            self.get_logger().info("正在发送请求到阶跃星辰语音识别API...")
            response = requests.post(
                self.asr_url,
                headers=headers,
                files=files,
                data=data,
                proxies=proxies
            )
            
            # 关闭文件
            files['file'].close()
            
            # 删除临时文件
            try:
                os.unlink(temp_wav_path)
            except Exception as e:
                self.get_logger().warning(f"删除临时文件失败: {str(e)}")
            
            # 检查响应
            if response.status_code == 200:
                try:
                    result = response.json()
                    if 'text' in result:
                        recognized_text = result['text']
                        self.get_logger().info(f"识别结果: {recognized_text}")
                        
                        # 发布识别结果
                        msg = String()
                        msg.data = recognized_text
                        self.speech_to_text_publisher.publish(msg)
                    else:
                        self.get_logger().error(f"API响应中没有文本结果: {result}")
                except Exception as e:
                    self.get_logger().error(f"解析API响应错误: {str(e)}")
            else:
                self.get_logger().error(f"API请求失败，状态码: {response.status_code}, 响应: {response.text}")
        
        except Exception as e:
            self.get_logger().error(f"语音识别过程中发生错误: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    
    node = SpeechRecognitionStepfunNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源
        if hasattr(node, 'raw_file') and node.raw_file:
            node.raw_file.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
