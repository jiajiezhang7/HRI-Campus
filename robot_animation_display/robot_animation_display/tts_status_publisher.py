#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
TTS状态发布节点 - 监控TTS状态并发布信息到/tts_status话题
这个节点通过监听TTS相关话题来判断机器人是否正在说话，并将状态发布到/tts_status话题
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Empty
from audio_common_msgs.msg import AudioData
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class TtsStatusPublisher(Node):
    def __init__(self):
        super().__init__('tts_status_publisher')

        # 配置QoS以确保消息可靠传递
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 创建发布者用于发布TTS状态
        self.status_publisher = self.create_publisher(
            Bool, 
            '/tts_status', 
            qos_profile
        )
        
        # 订阅TTS开始话题，以检测何时开始说话
        self.tts_start_subscription = self.create_subscription(
            AudioData,
            '/audio_generated',
            self.tts_start_callback,
            qos_profile
        )
        
        # 订阅TTS完成话题，以检测何时停止说话
        self.tts_done_subscription = self.create_subscription(
            Empty,
            '/audio_playback_complete',
            self.tts_done_callback,
            qos_profile
        )
        
        # 当前TTS状态
        self.is_speaking = False
        
        self.get_logger().info('TTS状态发布节点已启动，监听TTS状态并发布到/tts_status话题')
    
    def tts_start_callback(self, msg):
        """当收到音频生成消息时，将状态设置为'正在说话'"""
        if not self.is_speaking:
            self.is_speaking = True
            self.publish_status()
            self.get_logger().info('检测到音频生成，状态更新为：正在说话')
    
    def tts_done_callback(self, msg):
        """当收到音频播放完成消息时，将状态设置为'空闲'"""
        if self.is_speaking:
            self.is_speaking = False
            self.publish_status()
            self.get_logger().info('检测到音频播放完成，状态更新为：空闲')
    
    def publish_status(self):
        """发布当前TTS状态"""
        msg = Bool()
        msg.data = self.is_speaking
        self.status_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TtsStatusPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
