#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2节点，使用百度语音识别API进行语音识别
订阅音频数据并发布识别出的文本
"""

import os
import sys
import time
import wave
import json
import base64
import numpy as np
from urllib.request import urlopen, Request
from urllib.error import URLError
from urllib.parse import urlencode
import threading
import struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from audio_common_msgs.msg import AudioData

class SpeechRecognitionBaiduNode(Node):
    """
    使用百度语音识别API进行语音识别的ROS2节点
    """
    def __init__(self):
        super().__init__('speech_recognition_baidu_node')
        
        # 声明参数
        self.declare_parameter('language', 'zh-cn')  # 默认使用中文
        self.declare_parameter('buffer_size', 32000)  # 音频缓冲区大小
        self.declare_parameter('silence_threshold',50)  # 静音阈值
        self.declare_parameter('min_utterance_length', 16000)  # 最小语句长度（约0.5秒）
        self.declare_parameter('silence_duration', 1.5)  # 静音持续时间（秒）判定为句子结束
        self.declare_parameter('channels', 2)  # 声道数
        self.declare_parameter('sample_width', 2)  # 采样宽度（字节）
        self.declare_parameter('sample_rate', 16000)  # 采样率
        
        # 百度API参数
        self.declare_parameter('baidu_api_key', '')  # 百度API密钥
        self.declare_parameter('baidu_secret_key', '')  # 百度Secret密钥
        
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
        
        # 获取百度API参数
        self.api_key = self.get_parameter('baidu_api_key').value
        self.secret_key = self.get_parameter('baidu_secret_key').value
        
        # 获取网络代理参数
        self.use_proxy = self.get_parameter('use_proxy').value
        self.http_proxy = self.get_parameter('http_proxy').value
        self.https_proxy = self.get_parameter('https_proxy').value
        
        # 检查百度API参数是否设置
        if not self.api_key or not self.secret_key:
            self.get_logger().error("百度API参数未设置。请设置baidu_api_key和baidu_secret_key参数。")
        
        # 创建订阅者，订阅音频数据
        self.audio_subscription = self.create_subscription(
            AudioData,
            '/audio/audio',
            self.audio_callback,
            10
        )
        
        # 创建订阅者，订阅麦克风静音状态
        self.mute_subscription = self.create_subscription(
            Bool,
            '/mic_mute',
            self.mute_callback,
            10
        )
        
        # 麦克风状态
        self.mic_muted = False
        
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
        self.debug_dir = os.path.expanduser('~/speech_recognition_baidu_debug')
        os.makedirs(self.debug_dir, exist_ok=True)
        
        # 创建原始数据文件，用于调试
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        self.raw_file = open(os.path.join(self.debug_dir, f'raw_audio_{timestamp}.bin'), 'wb')
        self.get_logger().info(f'创建原始数据文件: {self.raw_file.name}')
        
        # 创建定时器，每10秒报告一次状态
        self.timer = self.create_timer(10.0, self.report_status)
        
        # 百度语音识别API的URL和参数
        self.asr_url = 'http://vop.baidu.com/server_api'
        self.dev_pid = 1537  # 普通话(支持简单的英文识别)
        self.cuid = '123456PYTHON'  # 用户唯一标识
        self.scope = 'audio_voice_assistant_get'  # 有此scope表示有asr能力
        
        self.get_logger().info(f'语音识别节点已初始化，使用语言: {self.language}')
        self.get_logger().info(f'音频格式: {self.channels}声道, {self.sample_width*8}位, {self.sample_rate}Hz')
        self.get_logger().info('使用百度语音识别API (REST API方式)')
    
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
    
    def mute_callback(self, msg):
        """
        处理麦克风静音状态消息
        """
        self.mic_muted = msg.data
        if self.mic_muted:
            self.get_logger().info('麦克风已静音，暂停音频处理')
        else:
            self.get_logger().info('麦克风已恢复，继续音频处理')
    
    def audio_callback(self, msg):
        """
        处理接收到的音频数据，使用VAD检测完整句子
        """
        # 如果麦克风静音，忽略音频数据
        if self.mic_muted:
            return
            
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
    
    def fetch_token(self):
        """
        获取百度语音识别API的访问令牌
        """
        token_url = 'http://aip.baidubce.com/oauth/2.0/token'
        params = {
            'grant_type': 'client_credentials',
            'client_id': self.api_key,
            'client_secret': self.secret_key
        }
        post_data = urlencode(params).encode('utf-8')
        
        # 设置代理
        if self.use_proxy:
            if self.http_proxy:
                os.environ['http_proxy'] = self.http_proxy
            if self.https_proxy:
                os.environ['https_proxy'] = self.https_proxy
        
        req = Request(token_url, post_data)
        try:
            f = urlopen(req)
            result_str = f.read().decode()
            result = json.loads(result_str)
            if 'access_token' in result.keys():
                self.get_logger().info(f"获取token成功，有效期: {result['expires_in']}秒")
                return result['access_token']
            else:
                self.get_logger().error(f"获取token失败: {result_str}")
                return None
        except URLError as err:
            self.get_logger().error(f'token请求错误: {str(err)}')
            return None
    
    def recognize_speech(self, audio_data):
        """
        使用百度语音识别API进行语音识别
        """
        try:
            # 创建临时WAV文件
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            temp_filename = os.path.join(self.debug_dir, f'utterance_{timestamp}.wav')
            
            # 处理音频数据 - 确保是16k采样率、16位、单声道
            try:
                # 如果是立体声，转换为单声道
                if self.channels == 2:
                    self.get_logger().info("将立体声音频转换为单声道...")
                    audio_array = np.frombuffer(audio_data, dtype=np.int16)
                    audio_array = audio_array.reshape(-1, 2)
                    # 取两个声道的平均值
                    audio_array = np.mean(audio_array, axis=1, dtype=np.int16)
                    audio_data = audio_array.tobytes()
                    self.get_logger().info(f"转换后音频长度: {len(audio_data)} 字节")
                
                # 检查采样率
                if self.sample_rate != 16000:
                    self.get_logger().warn(f"当前采样率 {self.sample_rate}Hz 不是16kHz，可能影响识别效果")
            except Exception as e:
                self.get_logger().error(f"处理音频数据错误: {str(e)}")
            
            # 将音频数据写入WAV文件
            with wave.open(temp_filename, 'wb') as wf:
                wf.setnchannels(1)  # 单声道
                wf.setsampwidth(self.sample_width)  # 16位
                wf.setframerate(16000)  # 16kHz
                wf.writeframes(audio_data)
            
            self.get_logger().info(f'保存语音片段到: {temp_filename}')
            
            # 如果API参数未设置，仅保存音频文件，不进行识别
            if not self.api_key or not self.secret_key:
                self.get_logger().error("百度API参数未设置，无法进行识别")
                return
            
            # 获取访问令牌
            token = self.fetch_token()
            if not token:
                self.get_logger().error("获取token失败，无法进行识别")
                return
            
            # 准备音频数据
            speech_data = audio_data
            length = len(speech_data)
            if length == 0:
                self.get_logger().error('音频数据长度为0')
                return
            
            # Base64编码
            speech = base64.b64encode(speech_data).decode('utf-8')
            
            # 设置语言参数
            dev_pid = self.dev_pid
            if self.language == 'en-us':
                dev_pid = 1737  # 英语
            
            # 构建请求参数
            params = {
                'dev_pid': dev_pid,
                'format': 'pcm',
                'rate': 16000,
                'token': token,
                'cuid': self.cuid,
                'channel': 1,
                'speech': speech,
                'len': length
            }
            
            # 发送请求
            self.get_logger().info("发送语音识别请求...")
            post_data = json.dumps(params).encode('utf-8')
            req = Request(self.asr_url, post_data)
            req.add_header('Content-Type', 'application/json')
            
            try:
                start_time = time.time()
                f = urlopen(req)
                result_str = f.read().decode('utf-8')
                self.get_logger().info(f"请求耗时: {time.time() - start_time:.2f}秒")
                self.get_logger().info(f"识别结果: {result_str}")
                
                # 解析结果
                result = json.loads(result_str)
                if result['err_no'] == 0:
                    text = result['result'][0]
                    self.get_logger().info(f"识别成功: {text}")
                    
                    # 发布识别结果
                    msg = String()
                    msg.data = text
                    self.speech_to_text_publisher.publish(msg)
                    self.get_logger().info(f"已发布识别结果: {text}")
                else:
                    self.get_logger().error(f"识别失败: {result['err_msg']}")
            
            except URLError as err:
                self.get_logger().error(f'识别请求错误: {str(err)}')
            except Exception as e:
                self.get_logger().error(f'识别过程错误: {str(e)}')
            
        except Exception as e:
            self.get_logger().error(f'语音识别错误: {str(e)}')
    
    def report_status(self):
        """
        报告当前状态
        """
        if self.is_speaking:
            self.get_logger().info(f'当前正在收集语音，已收集 {len(self.utterance_buffer)/self.sample_rate/self.channels/self.sample_width:.1f} 秒')
        
        # 计算音频能量，检测是否有声音
        if len(self.audio_buffer) > 0:
            try:
                audio_array = np.frombuffer(self.audio_buffer, dtype=np.int16)
                energy = np.sqrt(np.mean(np.square(audio_array.astype(np.float32))))
                self.get_logger().info(f'当前音频能量: {energy:.2f}')
            except Exception as e:
                self.get_logger().error(f'计算音频能量错误: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = SpeechRecognitionBaiduNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if hasattr(node, 'raw_file') and node.raw_file:
            node.raw_file.close()
        
        node.get_logger().info('语音识别已停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
