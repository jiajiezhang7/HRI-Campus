#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
摄像头系统FlexBE状态
"""

import rclpy
from rclpy.duration import Duration
from std_msgs.msg import Bool, Float32

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached, ProxyPublisher

class CameraSystemState(EventState):
    """
    封装摄像头系统功能的状态。

    该状态启动摄像头系统，并等待其初始化完成。

    -- face_angle_topic  string    人脸角度话题名称
    -- continue_detection_topic string 继续检测标志话题名称
    -- timeout           float     超时时间（秒）

    <= initialized       初始化成功
    <= failed            初始化失败
    <= timeout           超时
    """

    def __init__(self, face_angle_topic='/face_angle',
                 continue_detection_topic='/continue_detection',
                 timeout=10.0):
        """初始化状态"""
        super(CameraSystemState, self).__init__(
            outcomes=['initialized', 'failed', 'timeout']
        )
        
        # 存储参数
        self._face_angle_topic = face_angle_topic
        self._continue_detection_topic = continue_detection_topic
        self._timeout = timeout
        
        # 状态标志
        self._initialized = False
        
        # 初始化代理
        ProxySubscriberCached.initialize(CameraSystemState._node)
        ProxyPublisher.initialize(CameraSystemState._node)
        
        # 创建订阅者
        self._face_angle_sub = ProxySubscriberCached()
        self._face_angle_sub.subscribe(face_angle_topic, Float32)
        
        # 创建发布者
        self._detection_pub = ProxyPublisher()

    def execute(self, userdata):
        """
        执行状态逻辑
        
        该方法会被周期性调用，直到返回一个结果
        """
        # 检查超时
        if (self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds) > (self._timeout * 1e9):
            return 'timeout'
        
        # 检查是否收到人脸角度消息
        if self._face_angle_sub.has_msg(self._face_angle_topic):
            Logger.loginfo('摄像头系统初始化成功')
            self._initialized = True
            return 'initialized'
        
        # 继续等待初始化
        return None

    def on_enter(self, userdata):
        """
        当状态被激活时调用
        """
        self._start_time = self._node.get_clock().now()
        
        # 重置状态标志
        self._initialized = False
        
        # 发布继续检测标志
        msg = Bool()
        msg.data = True
        self._detection_pub.publish(self._continue_detection_topic, msg)
        
        Logger.loginfo('开始初始化摄像头系统')

    def on_exit(self, userdata):
        """
        当状态退出时调用
        """
        Logger.loginfo('退出摄像头系统状态')
