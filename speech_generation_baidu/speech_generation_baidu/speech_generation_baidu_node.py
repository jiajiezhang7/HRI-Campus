#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2节点，使用百度语音合成API将文本转换为语音
订阅LLM响应文本并发布合成的音频数据
"""

import os
import sys
import json
import base64
import time
from urllib.request import urlopen, Request
from urllib.error import URLError
from urllib.parse import urlencode, quote_plus

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData

class SpeechGenerationBaiduNode(Node):
    """
    使用百度语音合成API进行语音合成的ROS2节点
    """
    def __init__(self):
        super().__init__('speech_generation_baidu_node')
        
        # 声明参数
        self.declare_parameter('per', 4)  # 发音人选择，默认为度丫丫
        self.declare_parameter('spd', 5)  # 语速，范围0-15，默认5中语速
        self.declare_parameter('pit', 5)  # 音调，范围0-15，默认5中音调
        self.declare_parameter('vol', 5)  # 音量，范围0-15，默认5中音量
        self.declare_parameter('aue', 3)  # 文件格式，3:mp3, 4:pcm-16k, 5:pcm-8k, 6:wav
        
        # 百度API参数
        self.declare_parameter('baidu_api_key', '')  # 百度API密钥
        self.declare_parameter('baidu_secret_key', '')  # 百度Secret密钥
        
        # 网络代理参数
        self.declare_parameter('use_proxy', False)  # 是否使用代理
        self.declare_parameter('http_proxy', '')    # HTTP代理
        self.declare_parameter('https_proxy', '')   # HTTPS代理
        
        # 获取参数
        self.per = self.get_parameter('per').value
        self.spd = self.get_parameter('spd').value
        self.pit = self.get_parameter('pit').value
        self.vol = self.get_parameter('vol').value
        self.aue = self.get_parameter('aue').value
        
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
        
        # 创建调试文件目录
        self.debug_dir = os.path.expanduser('~/speech_generation_baidu_debug')
        os.makedirs(self.debug_dir, exist_ok=True)
        
        # 设置百度语音合成API的相关信息
        self.tts_url = 'http://tsn.baidu.com/text2audio'
        self.token_url = 'http://aip.baidubce.com/oauth/2.0/token'
        self.scope = 'audio_tts_post'  # 有此scope表示有tts能力
        self.format = {3: "mp3", 4: "pcm", 5: "pcm", 6: "wav"}[self.aue]
        self.cuid = "123456PYTHON"
        
        # 配置代理
        if self.use_proxy:
            if self.http_proxy:
                os.environ['http_proxy'] = self.http_proxy
            if self.https_proxy:
                os.environ['https_proxy'] = self.https_proxy
        
        self.get_logger().info(f'语音合成节点已初始化')
        self.get_logger().info(f'使用百度语音合成API (REST API方式)')
        self.get_logger().info(f'发音人: {self.per}, 语速: {self.spd}, 音调: {self.pit}, 音量: {self.vol}')
        self.get_logger().info(f'输出格式: {self.format}')
    
    def fetch_token(self):
        """
        获取百度语音合成API的访问令牌
        """
        self.get_logger().info("获取百度语音合成API的访问令牌")
        params = {
            'grant_type': 'client_credentials',
            'client_id': self.api_key,
            'client_secret': self.secret_key
        }
        post_data = urlencode(params)
        post_data = post_data.encode('utf-8')
        
        req = Request(self.token_url, post_data)
        try:
            f = urlopen(req, timeout=5)
            result_str = f.read()
            result_str = result_str.decode()
            
            self.get_logger().debug(f"获取Token响应: {result_str}")
            result = json.loads(result_str)
            
            if 'access_token' in result.keys() and 'scope' in result.keys():
                if self.scope not in result['scope'].split(' '):
                    self.get_logger().error('scope不正确')
                    return None
                
                self.get_logger().info(f"成功获取Token: {result['access_token']}, 有效期: {result['expires_in']}秒")
                return result['access_token']
            else:
                self.get_logger().error(f"获取Token失败，请检查API_KEY和SECRET_KEY是否正确")
                return None
        except URLError as err:
            self.get_logger().error(f"获取Token时网络错误: {str(err)}")
            return None
        except Exception as e:
            self.get_logger().error(f"获取Token时发生错误: {str(e)}")
            return None
    
    def text_to_speech(self, text):
        """
        将文本转换为语音
        """
        self.get_logger().info(f"开始将文本转换为语音: {text[:30]}...")
        
        try:
            # 获取token
            token = self.fetch_token()
            if not token:
                self.get_logger().error("无法获取Token，语音合成失败")
                return None
            
            # 对文本进行URL编码
            tex = quote_plus(text)
            
            # 构建参数
            params = {
                'tok': token,
                'tex': tex,
                'per': self.per,
                'spd': self.spd,
                'pit': self.pit,
                'vol': self.vol,
                'aue': self.aue,
                'cuid': self.cuid,
                'lan': 'zh',  # 固定参数
                'ctp': 1       # 固定参数
            }
            
            data = urlencode(params)
            req = Request(self.tts_url, data.encode('utf-8'))
            
            self.get_logger().debug(f"请求百度语音合成API: {self.tts_url}?{data}")
            
            has_error = False
            try:
                f = urlopen(req)
                result_str = f.read()
                
                headers = dict((name.lower(), value) for name, value in f.headers.items())
                has_error = ('content-type' not in headers.keys() or headers['content-type'].find('audio/') < 0)
                
            except URLError as err:
                self.get_logger().error(f"语音合成API请求错误: {str(err)}")
                result_str = err.read()
                has_error = True
            
            if has_error:
                # 保存错误信息用于调试
                timestamp = time.strftime("%Y%m%d-%H%M%S")
                error_file = os.path.join(self.debug_dir, f'error_{timestamp}.txt')
                with open(error_file, 'wb') as of:
                    of.write(result_str)
                
                try:
                    error_text = result_str.decode('utf-8')
                    self.get_logger().error(f"语音合成API错误: {error_text}")
                except:
                    self.get_logger().error(f"语音合成API发生未知错误，详情请查看: {error_file}")
                
                return None
            else:
                self.get_logger().info(f"语音合成成功，获取到{len(result_str)}字节的音频数据")
                
                # 保存音频文件用于调试
                timestamp = time.strftime("%Y%m%d-%H%M%S")
                audio_file = os.path.join(self.debug_dir, f'audio_{timestamp}.{self.format}')
                with open(audio_file, 'wb') as of:
                    of.write(result_str)
                
                self.get_logger().info(f"已保存音频文件用于调试: {audio_file}")
                return result_str
        
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
    
    node = SpeechGenerationBaiduNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
