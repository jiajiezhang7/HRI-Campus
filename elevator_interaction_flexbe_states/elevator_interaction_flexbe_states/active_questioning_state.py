#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
主动发问FlexBE状态
"""

import rclpy
from rclpy.duration import Duration
from std_msgs.msg import String
from std_srvs.srv import Empty

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller, ProxyPublisher, ProxySubscriberCached

class ActiveQuestioningState(EventState):
    """
    封装主动发问功能的状态。

    该状态触发主动发问服务，并等待LLM响应。

    -- active_questioning_service string 主动发问服务名称
    -- llm_response_topic string    LLM响应话题名称
    -- timeout           float     超时时间（秒）

    <= question_asked    问题已提出
    <= service_unavailable 服务不可用
    <= timeout           超时
    """

    def __init__(self, active_questioning_service='/active_questioning/trigger_question',
                 llm_response_topic='/llm_response',
                 timeout=10.0):
        """初始化状态"""
        super(ActiveQuestioningState, self).__init__(
            outcomes=['question_asked', 'service_unavailable', 'timeout']
        )
        
        # 存储参数
        self._active_questioning_service = active_questioning_service
        self._llm_response_topic = llm_response_topic
        self._timeout = timeout
        
        # 状态标志
        self._question_asked = False
        self._service_called = False
        
        # 初始化代理
        ProxyServiceCaller.initialize(ActiveQuestioningState._node)
        ProxySubscriberCached.initialize(ActiveQuestioningState._node)
        
        # 创建服务客户端
        self._trigger_question_client = ProxyServiceCaller()
        self._trigger_question_client.create_client(active_questioning_service, Empty)
        
        # 创建订阅者
        self._llm_response_sub = ProxySubscriberCached()
        self._llm_response_sub.subscribe(llm_response_topic, String)

    def execute(self, userdata):
        """
        执行状态逻辑
        
        该方法会被周期性调用，直到返回一个结果
        """
        # 检查超时
        if (self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds) > (self._timeout * 1e9):
            return 'timeout'
            
        # 检查服务是否可用
        if not self._trigger_question_client.is_available(self._active_questioning_service):
            Logger.logwarn('active_questioning服务不可用')
            return 'service_unavailable'
        
        # 调用服务
        if not self._service_called:
            Logger.loginfo('触发主动发问')
            request = Empty.Request()
            self._trigger_question_client.call_async(self._active_questioning_service, request,
                                                  lambda result: self._questioning_callback(result))
            self._service_called = True
        
        # 检查是否收到LLM响应
        if self._llm_response_sub.has_msg(self._llm_response_topic):
            msg = self._llm_response_sub.get_last_msg(self._llm_response_topic)
            self._llm_response_sub.remove_last_msg(self._llm_response_topic)
            
            response_text = msg.data
            Logger.loginfo(f'收到LLM响应: {response_text}')
            self._question_asked = True
            return 'question_asked'
        
        # 如果已经调用了服务但尚未收到响应，继续等待
        return None

    def on_enter(self, userdata):
        """
        当状态被激活时调用
        """
        self._start_time = self._node.get_clock().now()
        
        # 重置状态标志
        self._question_asked = False
        self._service_called = False
        
        Logger.loginfo('进入主动发问状态')

    def on_exit(self, userdata):
        """
        当状态退出时调用
        """
        Logger.loginfo('退出主动发问状态')

    def _questioning_callback(self, result):
        """
        主动发问服务调用完成的回调
        """
        try:
            Logger.loginfo('主动发问已触发')
        except Exception as e:
            Logger.logerr(f'触发主动发问时出错: {str(e)}')
