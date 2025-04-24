#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
等待人脸检测的FlexBE状态
"""

import rclpy
from rclpy.duration import Duration
from std_msgs.msg import Float32
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached

class WaitForFaceState(EventState):
    """
    等待人脸检测topic有消息再转移

    -- face_angle_topic string 人脸角度话题名称
    -- timeout float 超时时间（秒）

    <= detected 检测到人脸
    <= timeout 超时
    """
    def __init__(self, face_angle_topic='/face_angle', timeout=30.0):
        super(WaitForFaceState, self).__init__(
            outcomes=['detected', 'timeout']
        )
        self._face_angle_topic = face_angle_topic
        self._timeout = timeout
        self._start_time = None
        ProxySubscriberCached.initialize(WaitForFaceState._node)
        self._face_angle_sub = ProxySubscriberCached()
        self._face_angle_sub.subscribe(face_angle_topic, Float32)

    def on_enter(self, userdata):
        self._start_time = self._node.get_clock().now()
        Logger.loginfo(f'等待人脸检测消息，topic: {self._face_angle_topic}')

    def execute(self, userdata):
        if (self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds) > (self._timeout * 1e9):
            Logger.logwarn('等待人脸检测超时')
            return 'timeout'
        if self._face_angle_sub.has_msg(self._face_angle_topic):
            msg = self._face_angle_sub.get_last_msg(self._face_angle_topic)
            self._face_angle_sub.remove_last_msg(self._face_angle_topic)
            Logger.loginfo(f'检测到人脸，角度: {msg.data}')
            return 'detected'
        return None

    def on_exit(self, userdata):
        Logger.loginfo('退出等待人脸检测状态')
