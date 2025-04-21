#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
动画显示FlexBE状态
"""

import rclpy
from rclpy.duration import Duration
from std_msgs.msg import String, Bool

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached, ProxyPublisher

class AnimationDisplayState(EventState):
    """
    封装动画显示功能的状态。

    该状态启动动画显示系统，并等待其初始化完成。

    -- animation_status_topic string 动画状态话题名称
    -- timeout           float     超时时间（秒）

    <= initialized       初始化成功
    <= failed            初始化失败
    <= timeout           超时
    """

    def __init__(self, animation_status_topic='/animation_status',
                 timeout=10.0):
        """初始化状态"""
        super(AnimationDisplayState, self).__init__(
            outcomes=['initialized', 'failed', 'timeout']
        )
        
        # 存储参数
        self._animation_status_topic = animation_status_topic
        self._timeout = timeout
        
        # 状态标志
        self._initialized = False
        
        # 初始化代理
        ProxySubscriberCached.initialize(AnimationDisplayState._node)
        
        # 创建订阅者
        self._animation_status_sub = ProxySubscriberCached()
        self._animation_status_sub.subscribe(animation_status_topic, Bool)

    def execute(self, userdata):
        """
        执行状态逻辑
        
        该方法会被周期性调用，直到返回一个结果
        """
        # 检查超时
        if (self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds) > (self._timeout * 1e9):
            return 'timeout'
        
        # 检查是否收到动画状态消息
        if self._animation_status_sub.has_msg(self._animation_status_topic):
            msg = self._animation_status_sub.get_last_msg(self._animation_status_topic)
            self._animation_status_sub.remove_last_msg(self._animation_status_topic)
            
            if msg.data:
                Logger.loginfo('动画显示系统初始化成功')
                self._initialized = True
                return 'initialized'
            else:
                Logger.logwarn('动画显示系统初始化失败')
                return 'failed'
        
        # 继续等待初始化
        return None

    def on_enter(self, userdata):
        """
        当状态被激活时调用
        """
        self._start_time = self._node.get_clock().now()
        
        # 重置状态标志
        self._initialized = False
        
        Logger.loginfo('开始初始化动画显示系统')

    def on_exit(self, userdata):
        """
        当状态退出时调用
        """
        Logger.loginfo('退出动画显示状态')
