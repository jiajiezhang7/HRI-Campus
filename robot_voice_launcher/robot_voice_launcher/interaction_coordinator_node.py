#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
交互协调器节点，用于协调摄像头系统与语音交互系统
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
from std_srvs.srv import Empty
import time

class InteractionCoordinatorNode(Node):
    """
    交互协调器节点，管理摄像头系统与语音交互的工作流
    
    订阅话题:
        /face_angle: 从human_detect接收面向相机的人脸角度
        /llm_response: 接收LLM生成的响应文本
    
    发布话题:
        /continue_detection: 控制是否继续检测人脸
    """
    def __init__(self):
        super().__init__('interaction_coordinator_node')
        
        # 创建订阅者，接收人脸角度信息
        self.face_angle_sub = self.create_subscription(
            Float32,
            '/face_angle',
            self.face_angle_callback,
            10
        )
        
        # 创建订阅者，接收LLM响应
        self.llm_response_sub = self.create_subscription(
            String,
            '/llm_response',
            self.llm_response_callback,
            10
        )
        
        # 创建发布者，发布继续检测标志
        self.detection_pub = self.create_publisher(
            Bool,
            '/continue_detection',
            10
        )
        
        # 状态标志
        self.questioning_active = False  # 标记是否已经触发了主动发问
        self.interaction_complete = False  # 标记交互是否已完成
        
        # 创建客户端，用于调用active_questioning节点的服务
        self.trigger_question_client = self.create_client(
            Empty,
            '/active_questioning/trigger_question'
        )
        
        # 等待服务可用
        self.get_logger().info('等待active_questioning服务可用...')
        self.trigger_question_client.wait_for_service(timeout_sec=5.0)
        if not self.trigger_question_client.service_is_ready():
            self.get_logger().warn('active_questioning服务不可用，将在检测到人脸时重试')
        
        # 创建一个定时器，定期发布继续检测标志
        self.timer = self.create_timer(1.0, self.publish_detection_status)
        
        self.get_logger().info('交互协调器节点已初始化')
    
    def face_angle_callback(self, msg):
        """
        处理人脸角度消息
        当检测到人脸时触发主动发问
        """
        if self.questioning_active or self.interaction_complete:
            # 如果已经触发了主动发问或交互已完成，则忽略后续的人脸检测
            return
        
        face_angle = msg.data
        self.get_logger().info(f'检测到人脸，角度: {face_angle}°')
        
        # 触发主动发问
        self.trigger_questioning()
    
    def trigger_questioning(self):
        """
        触发主动发问服务
        """
        if not self.trigger_question_client.service_is_ready():
            self.get_logger().warn('active_questioning服务不可用，尝试重新连接...')
            self.trigger_question_client.wait_for_service(timeout_sec=1.0)
            if not self.trigger_question_client.service_is_ready():
                self.get_logger().error('无法连接到active_questioning服务')
                return
        
        self.get_logger().info('触发主动发问')
        
        # 标记已触发主动发问
        self.questioning_active = True
        
        # 创建请求
        request = Empty.Request()
        
        # 异步调用服务
        future = self.trigger_question_client.call_async(request)
        future.add_done_callback(self.questioning_callback)
    
    def questioning_callback(self, future):
        """
        主动发问服务调用完成的回调
        """
        try:
            response = future.result()
            self.get_logger().info('主动发问已触发')
        except Exception as e:
            self.get_logger().error(f'触发主动发问时出错: {str(e)}')
            # 如果出错，重置状态以便下次重试
            self.questioning_active = False
    
    def llm_response_callback(self, msg):
        """
        处理LLM响应消息
        根据响应内容决定是继续寻找人还是停止流程
        """
        if not self.questioning_active or self.interaction_complete:
            # 如果尚未触发主动发问或交互已完成，则忽略LLM响应
            return
        
        response_text = msg.data
        self.get_logger().info(f'收到LLM响应: {response_text}')
        
        # 根据响应内容决定是继续寻找人还是停止流程
        if response_text == "Thanks for your help":
            self.get_logger().info('检测到积极响应，停止交互')
            self.interaction_complete = True
        elif response_text == "Never mind, I will find someone else to help":
            self.get_logger().info('检测到消极响应，继续寻找人')
            # 重置状态，继续寻找人
            self.questioning_active = False
    
    def publish_detection_status(self):
        """
        定期发布继续检测标志
        """
        msg = Bool()
        # 如果交互已完成，则停止检测
        msg.data = not self.interaction_complete
        self.detection_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = InteractionCoordinatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
