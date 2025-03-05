#!/usr/bin/env python3

"""
服务触发器节点，用于在启动后自动调用active_questioning服务
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import time

class ServiceTriggerNode(Node):
    """服务触发器节点，用于在启动后自动调用服务"""
    
    def __init__(self):
        super().__init__('service_trigger_node')
        
        # 声明参数
        self.declare_parameter('service_name', '/active_questioning/trigger_question')
        self.declare_parameter('delay', 3.0)
        
        # 获取参数
        self.service_name = self.get_parameter('service_name').value
        self.delay = self.get_parameter('delay').value
        
        # 创建定时器，延迟后调用服务
        self.timer = self.create_timer(self.delay, self.trigger_service)
        self.timer_called = False
        
        self.get_logger().info(f'服务触发器节点已启动，将在{self.delay}秒后调用服务: {self.service_name}')
        
        # 创建服务客户端
        self.client = self.create_client(Empty, self.service_name)
    
    def trigger_service(self):
        """触发服务"""
        if self.timer_called:
            return
        
        self.timer_called = True
        self.timer.cancel()  # 只调用一次
        
        # 等待服务可用
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'服务 {self.service_name} 不可用')
            return
        
        # 调用服务
        request = Empty.Request()
        future = self.client.call_async(request)
        future.add_done_callback(self.service_callback)
        
        self.get_logger().info(f'已调用服务: {self.service_name}')
    
    def service_callback(self, future):
        """服务调用回调"""
        try:
            response = future.result()
            self.get_logger().info('服务调用成功')
        except Exception as e:
            self.get_logger().error(f'服务调用失败: {str(e)}')

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    node = ServiceTriggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
