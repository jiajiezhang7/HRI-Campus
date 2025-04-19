#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
人体检测FlexBE状态
"""

import rclpy
from rclpy.duration import Duration
from std_msgs.msg import Bool, Float32

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached, ProxyPublisher

class HumanDetectionState(EventState):
    """
    封装人体检测功能的状态。

    该状态监控人体检测结果，并在检测到人时触发相应的行为。

    -- face_angle_topic  string    人脸角度话题名称
    -- continue_detection_topic string 继续检测标志话题名称
    -- timeout           float     超时时间（秒）

    <= detected          检测到人
    <= not_detected      未检测到人
    <= timeout           超时
    """

    def __init__(self, face_angle_topic='/face_angle',
                 continue_detection_topic='/continue_detection',
                 timeout=60.0):
        """初始化状态"""
        super(HumanDetectionState, self).__init__(
            outcomes=['detected', 'not_detected', 'timeout']
        )
        
        # 存储参数
        self._face_angle_topic = face_angle_topic
        self._continue_detection_topic = continue_detection_topic
        self._timeout = timeout
        
        # 状态标志
        self._detection_enabled = True
        self._human_detected = False
        
        # 初始化代理
        ProxySubscriberCached.initialize(HumanDetectionState._node)
        ProxyPublisher.initialize(HumanDetectionState._node)
        
        # 创建订阅者
        self._face_angle_sub = ProxySubscriberCached()
        self._face_angle_sub.create_subscription(face_angle_topic, Float32)
        
        self._continue_detection_sub = ProxySubscriberCached()
        self._continue_detection_sub.create_subscription(continue_detection_topic, Bool)

    def execute(self, userdata):
        """
        执行状态逻辑
        
        该方法会被周期性调用，直到返回一个结果
        """
        # 检查超时
        if (self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds) > (self._timeout * 1e9):
            return 'timeout'
            
        # 检查是否应该继续检测
        if self._continue_detection_sub.has_msg(self._continue_detection_topic):
            msg = self._continue_detection_sub.get_last_msg(self._continue_detection_topic)
            self._continue_detection_sub.remove_last_msg(self._continue_detection_topic)
            
            self._detection_enabled = msg.data
            if not self._detection_enabled:
                Logger.loginfo('人体检测已禁用')
                return 'not_detected'
        
        # 检查是否检测到人脸
        if self._face_angle_sub.has_msg(self._face_angle_topic) and self._detection_enabled:
            msg = self._face_angle_sub.get_last_msg(self._face_angle_topic)
            self._face_angle_sub.remove_last_msg(self._face_angle_topic)
            
            face_angle = msg.data
            Logger.loginfo(f'检测到人脸，角度: {face_angle}°')
            self._human_detected = True
            return 'detected'
        
        # 继续检测
        return None

    def on_enter(self, userdata):
        """
        当状态被激活时调用
        """
        self._start_time = self._node.get_clock().now()
        
        # 重置状态标志
        self._detection_enabled = True
        self._human_detected = False
        
        Logger.loginfo('开始人体检测')

    def on_exit(self, userdata):
        """
        当状态退出时调用
        """
        Logger.loginfo('退出人体检测状态')
