#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
模拟电梯楼层信息发布节点
"""

import rclpy
from rclpy.node import Node
from level_interfaces.msg import Level
import random
import time


class DummyLevelPublisherNode(Node):
    """
    模拟电梯楼层信息发布节点
    发布电梯楼层和方向信息
    每次启动时生成一个随机的电梯信息，之后一直发布相同的信息
    """
    def __init__(self):
        super().__init__('dummy_level_publisher_node')
        
        # 创建发布者，发布电梯楼层信息
        self.level_publisher = self.create_publisher(
            Level,
            '/dummy_level',
            10
        )
        
        # 参数声明
        self.declare_parameter('publish_frequency', 5.0)  # 发布频率（秒）
        
        # 获取参数
        self.publish_frequency = self.get_parameter('publish_frequency').value
        
        # 初始化时随机生成固定的电梯方向和楼层
        self.fixed_direction = random.choice([True, False])
        self.fixed_level = random.randint(1, 20)
        
        direction_str = "上行" if self.fixed_direction else "下行"
        self.get_logger().info(f'生成固定电梯信息: {direction_str}, 楼层: {self.fixed_level}')
        
        # 创建定时器
        self.timer = self.create_timer(
            1.0 / self.publish_frequency,
            self.timer_callback
        )
        
        self.get_logger().info('模拟电梯楼层信息发布节点已初始化')
    
    def timer_callback(self):
        """
        定时发布固定的电梯楼层信息
        """
        # 创建消息并发布
        msg = Level()
        
        # 使用固定的电梯方向和楼层
        msg.is_up = self.fixed_direction
        msg.level = self.fixed_level
        
        self.level_publisher.publish(msg)
        
        direction = "上行" if msg.is_up else "下行"
        self.get_logger().debug(f'发布固定电梯信息: {direction}, 楼层: {msg.level}')


def main(args=None):
    rclpy.init(args=args)
    
    node = DummyLevelPublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
