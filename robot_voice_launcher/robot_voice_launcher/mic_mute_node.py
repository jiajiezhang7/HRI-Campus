#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2节点，用于在音频播放时临时禁用麦克风，防止扬声器声音被麦克风拾取导致的反馈循环
"""

import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty
from audio_common_msgs.msg import AudioData

class MicMuteNode(Node):
    """麦克风静音控制节点，监听音频播放状态，在播放时禁用麦克风"""
    
    def __init__(self):
        super().__init__('mic_mute_node')
        
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
        self.playback_complete_subscription = self.create_subscription(
            Empty,
            '/audio_playback_complete',
            self.playback_complete_callback,
            10
        )
        
        # 创建订阅者，直接订阅麦克风静音控制信号
        self.mute_control_subscription = self.create_subscription(
            Bool,
            '/mic_mute',
            self.mute_control_callback,
            1  # 设置高优先级
        )
        
        # 创建定时器，定期检查音频播放状态
        self.check_timer = self.create_timer(0.5, self.check_audio_status)
        
        self.get_logger().info('麦克风静音控制节点已初始化')
        
        # 初始状态：麦克风启用
        self.publish_mute_state(False)
        
    def mute_control_callback(self, msg):
        """
        处理直接的麦克风静音控制信号
        """
        if msg.data:
            # self.get_logger().info('收到静音信号，静音麦克风')
            with self.audio_lock:
                self.audio_playing = True
                self.last_audio_time = time.time()
            self.publish_mute_state(True)
        else:
            # self.get_logger().info('收到解除静音信号')
            with self.audio_lock:
                self.audio_playing = False
            self.publish_mute_state(False)  # 直接解除静音，不使用 unmute_microphone 方法
    
    def audio_callback(self, msg):
        """
        处理接收到的音频数据，当有音频播放时静音麦克风
        """
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
        self.get_logger().info('收到音频播放完成事件')
        with self.audio_lock:
            self.audio_playing = False
            self.get_logger().info(f'设置 audio_playing = False, 当前 is_muted = {self.is_muted}')
        
        # 音频播放完成，立即恢复麦克风（不在锁内调用，避免死锁）
        # 直接发布解除静音状态，不使用 unmute_microphone 方法
        self.get_logger().info('音频播放完成，直接解除麦克风静音')
        self.publish_mute_state(False)
    
    def check_audio_status(self):
        """
        定期检查音频播放状态，处理可能的播放完成事件丢失情况
        """
        with self.audio_lock:
            # 如果超过3秒没有收到音频数据，认为播放已结束
            if self.audio_playing and time.time() - self.last_audio_time > 3.0:
                self.get_logger().info('超过3秒未收到音频数据，认为播放已结束')
                self.audio_playing = False
        
        # 如果检测到播放结束，直接解除静音（不在锁内调用，避免死锁）
        if not self.audio_playing and self.is_muted:
            self.get_logger().info('检测到播放结束，直接解除麦克风静音')
            self.publish_mute_state(False)
    
    def unmute_microphone(self):
        """
        恢复麦克风（此方法保留但不再使用，为了兼容性）
        """
        self.get_logger().info(f'尝试解除麦克风静音，当前状态: is_muted={self.is_muted}, audio_playing={self.audio_playing}')
        
        # 直接解除静音，不再进行条件判断
        if self.is_muted:
            self.get_logger().info('解除麦克风静音')
            self.publish_mute_state(False)
        else:
            self.get_logger().info('麦克风已经处于非静音状态')
        
        # 清除定时器（为了兼容性保留这行代码）
        self.mute_timer = None
    
    def publish_mute_state(self, mute_state):
        """
        发布麦克风静音状态
        """
        self.is_muted = mute_state
        msg = Bool()
        msg.data = mute_state
        self.mute_publisher.publish(msg)
        # self.get_logger().info(f'已发布麦克风状态: {"静音" if mute_state else "非静音"}')


def main(args=None):
    rclpy.init(args=args)
    node = MicMuteNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
