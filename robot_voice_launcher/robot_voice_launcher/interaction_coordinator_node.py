#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
交互协调器节点，用于协调摄像头系统与语音交互系统
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool, Empty
from std_srvs.srv import Empty as EmptyService
import time

class InteractionCoordinatorNode(Node):
    """
    交互协调器节点，管理摄像头系统与语音交互的工作流
    
    订阅话题:
        /face_angle: 从human_detect接收面向相机的人脸角度
        /llm_response: 接收LLM生成的响应文本
        /audio_playback_complete: 接收音频播放完成事件
    
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
        
        # 创建订阅者，接收音频播放完成事件
        self.playback_complete_sub = self.create_subscription(
            Empty,  
            '/audio_playback_complete',
            self.playback_complete_callback,
            10
        )
        
        # 创建发布者，发布继续检测标志
        self.detection_pub = self.create_publisher(
            Bool,
            '/continue_detection',
            10
        )
        
        # 状态标志
        self.questioning_active = False  
        self.interaction_complete = False  
        self.waiting_for_playback = False  
        self.pending_reset = False  
        self.received_playback_event = False  
        
        # 创建客户端，用于调用active_questioning节点的服务
        self.trigger_question_client = self.create_client(
            EmptyService,  
            '/active_questioning/trigger_question'
        )
        
        # 等待服务可用
        self.get_logger().info('等待active_questioning服务可用...')
        self.trigger_question_client.wait_for_service(timeout_sec=5.0)
        if not self.trigger_question_client.service_is_ready():
            self.get_logger().warn('active_questioning服务不可用，将在检测到人脸时重试')
        
        # 创建一个定时器，定期发布继续检测标志
        self.timer = self.create_timer(1.0, self.publish_detection_status)
        
        # 发布初始状态，确保开始时能够检测人脸
        self.publish_initial_status()
        
        self.get_logger().info('交互协调器节点已初始化')
    
    def publish_initial_status(self):
        """
        发布初始状态，确保开始时能够检测人脸
        """
        msg = Bool()
        msg.data = True  
        self.detection_pub.publish(msg)
        self.get_logger().info('发布初始检测状态：允许检测')
    
    def face_angle_callback(self, msg):
        """
        处理人脸角度消息
        当检测到人脸时触发主动发问
        """
        if self.questioning_active or self.interaction_complete:
            return
        
        if self.waiting_for_playback and self.received_playback_event:
            return
        
        face_angle = msg.data
        self.get_logger().info(f'检测到人脸，角度: {face_angle}°')
        
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
        
        self.questioning_active = True
        
        request = EmptyService.Request()  
        
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
            self.questioning_active = False
    
    def llm_response_callback(self, msg):
        """
        处理LLM响应消息
        根据响应内容决定是继续寻找人还是停止流程
        """
        if not self.questioning_active or self.interaction_complete:
            return
        
        response_text = msg.data
        self.get_logger().info(f'收到LLM响应: {response_text}')
        
        positive_keywords = ["hero", "小可爱"]
        negative_keywords = ["jerk", "小趴菜"]
        
        if any(keyword in response_text for keyword in positive_keywords):
            self.get_logger().info('检测到积极响应，停止交互')
            self.interaction_complete = True
        elif any(keyword in response_text for keyword in negative_keywords):
            self.get_logger().info('检测到消极响应，等待音频播放完成后继续寻找人')
            self.waiting_for_playback = True
            self.pending_reset = True
    
    def playback_complete_callback(self, msg):
        """
        处理音频播放完成事件
        如果有待处理的状态重置，则重置状态
        """
        self.get_logger().info('音频播放完成')
        self.received_playback_event = True
        
        if self.waiting_for_playback and self.pending_reset:
            self.get_logger().info('重置状态，继续寻找人')
            self.waiting_for_playback = False
            self.pending_reset = False
            self.questioning_active = False
    
    def publish_detection_status(self):
        """
        定期发布继续检测标志
        """
        msg = Bool()
        
        if not self.received_playback_event:
            msg.data = not self.interaction_complete  
        else:
            msg.data = not (self.interaction_complete or self.waiting_for_playback)
            
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
