#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2节点，用于在音频播放时临时禁用麦克风，防止扬声器声音被麦克风拾取导致的反馈循环
"""

import rclpy
import threading
import time
from rclpy.node import Node
from std_msgs.msg import Bool, Empty
from audio_common_msgs.msg import AudioData


class MicMuteNode(Node):
    """
    麦克风静音控制节点，监听音频播放状态，在播放时禁用麦克风
    """
    def __init__(self):
        super().__init__('mic_mute_node')
        
        # 声明参数
        self.declare_parameter('mute_duration', 7.0)  # 音频播放后保持麦克风静音的时间（秒）
        
        # 获取参数
        self.mute_duration = self.get_parameter('mute_duration').value
        
        # 状态变量
        self.is_muted = False
        self.mute_timer = None
        self.last_audio_time = 0.0
        self.audio_playing = False
        self.audio_lock = threading.Lock()
        
        # 创建发布者，发布麦克风静音状态
        self.mute_publisher = self.create_publisher(
            Bool,
            '/mic_mute',
            10
        )
        
        # 创建订阅者，订阅音频播放数据
        self.audio_subscription = self.create_subscription(
            AudioData,
            '/audio_generated',
            self.audio_callback,
            10
        )
        
        # 创建订阅者，订阅音频播放完成事件
        # 注意：这需要音频播放节点发布此事件
        self.playback_complete_subscription = self.create_subscription(
            Empty,
            '/audio_playback_complete',
            self.playback_complete_callback,
            10
        )
        
        # 创建定时器，定期检查音频播放状态
        self.check_timer = self.create_timer(0.5, self.check_audio_status)
        
        self.get_logger().info('麦克风静音控制节点已初始化')
        self.get_logger().info(f'静音持续时间: {self.mute_duration}秒')
        
        # 初始状态：麦克风启用
        self.publish_mute_state(False)
    
    def audio_callback(self, msg):
        """
        处理接收到的音频数据，当有音频播放时静音麦克风
        """
        if not msg.data:
            return
        
        with self.audio_lock:
            self.last_audio_time = time.time()
            self.audio_playing = True
            
            # 检测到音频数据，静音麦克风
            if not self.is_muted:
                self.get_logger().info('检测到音频播放，静音麦克风')
                self.publish_mute_state(True)
    
    def playback_complete_callback(self, msg):
        """
        处理音频播放完成事件
        """
        with self.audio_lock:
            self.audio_playing = False
            
            # 设置定时器，在指定时间后恢复麦克风
            if self.mute_timer:
                self.mute_timer.cancel()
            
            self.mute_timer = self.create_timer(
                self.mute_duration,
                self.unmute_microphone
            )
    
    def check_audio_status(self):
        """
        定期检查音频播放状态，处理可能的播放完成事件丢失情况
        """
        with self.audio_lock:
            if self.audio_playing and time.time() - self.last_audio_time > 3.0:
                # 如果超过3秒没有收到新的音频数据，认为播放已完成
                self.audio_playing = False
                
                # 设置定时器，在指定时间后恢复麦克风
                if self.mute_timer:
                    self.mute_timer.cancel()
                
                self.mute_timer = self.create_timer(
                    self.mute_duration,
                    self.unmute_microphone
                )
    
    def unmute_microphone(self):
        """
        恢复麦克风
        """
        with self.audio_lock:
            if self.is_muted and not self.audio_playing:
                self.get_logger().info('音频播放结束，恢复麦克风')
                self.publish_mute_state(False)
            
            # 清除定时器
            self.mute_timer = None
    
    def publish_mute_state(self, mute_state):
        """
        发布麦克风静音状态
        """
        self.is_muted = mute_state
        msg = Bool()
        msg.data = mute_state
        self.mute_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MicMuteNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
