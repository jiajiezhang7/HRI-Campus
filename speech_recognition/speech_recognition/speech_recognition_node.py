#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2节点，使用科大讯飞多语种语音识别API进行语音识别
订阅音频数据并发布识别出的文本
"""

import os
import sys
import time
import wave
import json
import base64
import hmac
import hashlib
import uuid
import ssl
import datetime
import websocket
import numpy as np
from urllib.parse import urlencode, quote
from urllib.request import Request, urlopen
import threading
import struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData

class SpeechRecognitionNode(Node):
    """
    使用科大讯飞多语种语音识别API进行语音识别的ROS2节点
    """
    def __init__(self):
        super().__init__('speech_recognition_node')
        
        # 声明参数
        self.declare_parameter('language', 'zh-cn')  # 默认使用中文
        self.declare_parameter('buffer_size', 32000)  # 音频缓冲区大小
        self.declare_parameter('silence_threshold', 200)  # 静音阈值
        self.declare_parameter('min_utterance_length', 16000)  # 最小语句长度（约0.5秒）
        self.declare_parameter('silence_duration', 1.5)  # 静音持续时间（秒）判定为句子结束
        self.declare_parameter('channels', 2)  # 声道数
        self.declare_parameter('sample_width', 2)  # 采样宽度（字节）
        self.declare_parameter('sample_rate', 16000)  # 采样率
        
        # 科大讯飞API参数
        self.declare_parameter('xf_app_id', '')  # 讯飞应用ID
        self.declare_parameter('xf_api_key', '')  # 讯飞API密钥
        self.declare_parameter('xf_api_secret', '')  # 讯飞API Secret
        
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
        
        # 获取讯飞API参数
        self.xf_app_id = self.get_parameter('xf_app_id').value
        self.xf_api_key = self.get_parameter('xf_api_key').value
        self.xf_api_secret = self.get_parameter('xf_api_secret').value
        
        # 获取网络代理参数
        self.use_proxy = self.get_parameter('use_proxy').value
        self.http_proxy = self.get_parameter('http_proxy').value
        self.https_proxy = self.get_parameter('https_proxy').value
        
        # 检查讯飞API参数是否设置
        if not self.xf_app_id or not self.xf_api_key or not self.xf_api_secret:
            self.get_logger().error("科大讯飞API参数未设置。请设置xf_app_id, xf_api_key和xf_api_secret参数。")
        
        # 创建订阅者，订阅音频数据
        self.audio_subscription = self.create_subscription(
            AudioData,
            '/audio/audio',
            self.audio_callback,
            10
        )
        
        # 创建发布者，发布识别出的文本
        self.text_publisher = self.create_publisher(
            String,
            '/recognized_text',
            10
        )
        
        # 创建新的发布者，发布到/speech_to_text话题
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
        
        # WebSocket相关变量
        self.ws = None
        self.ws_connected = False
        self.ws_result = ""
        self.ws_lock = threading.Lock()
        
        # 创建调试文件目录
        self.debug_dir = os.path.expanduser('~/speech_recognition_debug')
        os.makedirs(self.debug_dir, exist_ok=True)
        
        # 创建原始数据文件，用于调试
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        self.raw_file = open(os.path.join(self.debug_dir, f'raw_audio_{timestamp}.bin'), 'wb')
        self.get_logger().info(f'创建原始数据文件: {self.raw_file.name}')
        
        # 创建定时器，每10秒报告一次状态
        self.timer = self.create_timer(10.0, self.report_status)
        
        self.get_logger().info(f'语音识别节点已初始化，使用语言: {self.language}')
        self.get_logger().info(f'音频格式: {self.channels}声道, {self.sample_width*8}位, {self.sample_rate}Hz')
        self.get_logger().info('使用科大讯飞多语种语音识别API (WebSocket方式)')
    
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
    
    def create_request_signature(self):
        """
        创建科大讯飞WebSocket API请求签名
        """
        # WebSocket API的URL
        base_url = "wss://iat.cn-huabei-1.xf-yun.com/v1"
        
        # 当前UTC时间戳，RFC1123格式
        now = datetime.datetime.now(datetime.timezone.utc)
        date = now.strftime('%a, %d %b %Y %H:%M:%S GMT')
        
        # 构建签名原始内容
        host = "iat.cn-huabei-1.xf-yun.com"
        request_line = "GET /v1 HTTP/1.1"
        signature_origin = f"host: {host}\ndate: {date}\n{request_line}"
        
        # 使用HMAC-SHA256计算签名
        signature_sha = hmac.new(
            self.xf_api_secret.encode('utf-8'),
            signature_origin.encode('utf-8'),
            digestmod=hashlib.sha256
        ).digest()
        
        # Base64编码
        signature = base64.b64encode(signature_sha).decode('utf-8')
        
        # 构建授权字符串
        authorization_origin = f'api_key="{self.xf_api_key}", algorithm="hmac-sha256", headers="host date request-line", signature="{signature}"'
        authorization = base64.b64encode(authorization_origin.encode('utf-8')).decode('utf-8')
        
        # 构建完整的URL，包含鉴权信息
        url = f"{base_url}?authorization={authorization}&date={quote(date)}&host={host}"
        
        return url
    
    def recognize_speech(self, audio_data):
        """
        使用科大讯飞多语种语音识别API进行语音识别
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
            if not self.xf_app_id or not self.xf_api_key or not self.xf_api_secret:
                self.get_logger().error("科大讯飞API参数未设置，无法进行识别")
                return
            
            # 配置讯飞API请求
            url = self.create_request_signature()
            
            # 配置代理设置
            proxies = {}
            if self.use_proxy:
                if self.http_proxy or self.https_proxy:
                    proxies = {
                        "http": self.http_proxy if self.http_proxy else None,
                        "https": self.https_proxy if self.https_proxy else None
                    }
                    self.get_logger().info(f"使用自定义代理: {proxies}")
                else:
                    # 使用系统代理
                    self.get_logger().info("使用系统默认代理")
                    proxies = None  # requests会自动使用系统代理
            else:
                # 禁用代理设置，直接连接
                proxies = {"http": None, "https": None}
                self.get_logger().info("已禁用代理，直接连接")
            
            try:
                self.get_logger().info("尝试连接WebSocket...")
                
                # 配置WebSocket连接选项
                ws_options = {
                    "sslopt": {"cert_reqs": ssl.CERT_NONE}
                }
                
                # 添加代理支持
                if self.use_proxy and (self.http_proxy or self.https_proxy):
                    proxy = self.https_proxy if self.https_proxy else self.http_proxy
                    if proxy:
                        # 解析代理地址和端口
                        if proxy.startswith('http://'):
                            proxy = proxy[7:]
                        elif proxy.startswith('https://'):
                            proxy = proxy[8:]
                        
                        # 分离主机和端口
                        if ':' in proxy:
                            proxy_host, proxy_port = proxy.split(':')
                            proxy_port = int(proxy_port)
                        else:
                            proxy_host = proxy
                            proxy_port = 80  # 默认HTTP代理端口
                        
                        self.get_logger().info(f"使用代理: {proxy_host}:{proxy_port}")
                        ws_options["http_proxy_host"] = proxy_host
                        ws_options["http_proxy_port"] = proxy_port
                
                # 连接WebSocket
                self.ws = websocket.WebSocket()
                self.ws.connect(url, **ws_options)
                self.get_logger().info("WebSocket连接成功")
                
                # 设置语言参数
                lang_code = "zh_cn"  # 默认为中文
                if self.language == "en-us":
                    lang_code = "en_us"
                
                # 发送开始识别请求
                self.get_logger().info("发送开始识别请求...")
                start_request = {
                    "header": {
                        "status": 0,  # 首帧，标识音频开始
                        "app_id": self.xf_app_id
                    },
                    "parameter": {
                        "iat": {
                            "domain": "slm",  # 领域
                            "language": lang_code,  # 语言
                            "accent": "mandarin",  # 方言，普通话
                            "dwa": "wpgs",  # 开启动态修正功能
                            "result": {
                                "encoding": "utf8",
                                "compress": "raw",
                                "format": "json"
                            }
                        }
                    },
                    "payload": {
                        "audio": {
                            "audio": "",  # 首帧不发送音频数据
                            "sample_rate": 16000,
                            "encoding": "raw"
                        }
                    }
                }
                self.get_logger().info(f"发送请求: {json.dumps(start_request, ensure_ascii=False)}")
                self.ws.send(json.dumps(start_request))
                
                # 发送音频数据
                self.get_logger().info("发送音频数据...")
                audio_base64 = base64.b64encode(audio_data).decode('utf-8')
                audio_request = {
                    "header": {
                        "status": 1,  # 中间帧，音频继续
                        "app_id": self.xf_app_id
                    },
                    "payload": {
                        "audio": {
                            "audio": audio_base64,
                            "sample_rate": 16000,
                            "encoding": "raw"
                        }
                    }
                }
                self.get_logger().info("发送音频数据帧...")
                self.ws.send(json.dumps(audio_request))
                
                # 发送结束识别请求
                self.get_logger().info("发送结束识别请求...")
                end_request = {
                    "header": {
                        "status": 2,  # 尾帧，标识音频结束
                        "app_id": self.xf_app_id
                    },
                    "payload": {
                        "audio": {
                            "audio": "",  # 尾帧不发送音频数据
                            "sample_rate": 16000,
                            "encoding": "raw"
                        }
                    }
                }
                self.get_logger().info("发送结束帧...")
                self.ws.send(json.dumps(end_request))
                
                # 接收识别结果
                self.get_logger().info("接收识别结果...")
                final_text = ""
                while True:
                    result = self.ws.recv()
                    self.get_logger().info(f"收到原始响应: {result}")
                    
                    try:
                        message = json.loads(result)
                        self.get_logger().info(f"响应JSON结构: {json.dumps(message, ensure_ascii=False)}")
                        
                        # 检查响应状态码
                        code = message["header"]["code"]
                        status = message["header"]["status"]
                        
                        if code != 0:
                            self.get_logger().error(f"请求错误: {code}")
                            break
                        
                        # 提取结果
                        payload = message.get("payload")
                        if payload and "result" in payload:
                            text = payload["result"]["text"]
                            # 解码base64文本
                            try:
                                text = json.loads(str(base64.b64decode(text), "utf8"))
                                text_ws = text['ws']
                                current_text = ''
                                for i in text_ws:
                                    for j in i["cw"]:
                                        w = j["w"]
                                        current_text += w
                                
                                if current_text:
                                    final_text += current_text
                                    self.get_logger().info(f"当前识别结果: {current_text}")
                            except Exception as e:
                                self.get_logger().error(f"解析结果文本错误: {str(e)}")
                        
                        # 如果是最后一个结果
                        if status == 2:
                            self.get_logger().info("识别完成")
                            break
                    
                    except json.JSONDecodeError as e:
                        self.get_logger().error(f"JSON解析错误: {str(e)}")
                        self.get_logger().info(f"无法解析的响应: {result}")
                        break
                    except KeyError as e:
                        self.get_logger().error(f"访问JSON字段错误: {str(e)}")
                        continue
                    except Exception as e:
                        self.get_logger().error(f"处理响应时出错: {str(e)}")
                        continue
                
                self.get_logger().info(f"最终识别结果: {final_text}")
                
                # 关闭WebSocket连接
                self.ws.close()
                
                # 发布识别结果到两个话题
                if final_text:
                    msg = String()
                    msg.data = final_text
                    self.text_publisher.publish(msg)
                    self.speech_to_text_publisher.publish(msg)
                    self.get_logger().info(f"已发布识别结果: {final_text}")
                else:
                    self.get_logger().warn("未能识别出文本")
                
            except websocket.WebSocketException as e:
                self.get_logger().error(f"WebSocket连接错误: {str(e)}")
            except Exception as e:
                self.get_logger().error(f"识别错误: {str(e)}")
            
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
    node = SpeechRecognitionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if hasattr(node, 'raw_file') and node.raw_file:
            node.raw_file.close()
        
        # 关闭WebSocket连接
        if hasattr(node, 'ws') and node.ws:
            try:
                node.ws.close()
                node.get_logger().info('WebSocket连接已关闭')
            except Exception as e:
                node.get_logger().error(f'关闭WebSocket连接错误: {str(e)}')
        
        node.get_logger().info('语音识别已停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
