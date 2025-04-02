#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
模拟TTS状态发布节点 - 用于测试动画显示系统
每隔几秒钟交替发布"说话"和"空闲"状态
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time

class MockTtsStatusPublisher(Node):
    def __init__(self):
        super().__init__('mock_tts_status_publisher')
        # 创建发布者，话题名必须和网页中的 TTS_STATUS_TOPIC 一致
        self.publisher_ = self.create_publisher(Bool, '/tts_status', 10)
        self.timer = self.create_timer(8, self.timer_callback) # 每8秒模拟一次说话/停止
        self.is_speaking = False
        self.get_logger().info('模拟TTS状态发布节点已启动，每8秒切换一次状态')

    def timer_callback(self):
        self.is_speaking = not self.is_speaking # 切换状态
        msg = Bool()
        msg.data = self.is_speaking
        self.publisher_.publish(msg)
        self.get_logger().info(f'发布状态: {"正在说话" if self.is_speaking else "空闲"}')

def main(args=None):
    rclpy.init(args=args)
    node = MockTtsStatusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
