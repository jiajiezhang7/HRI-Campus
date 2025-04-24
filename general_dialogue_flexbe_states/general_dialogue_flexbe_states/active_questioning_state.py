#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
主动发问FlexBE状态
"""

import rclpy
from rclpy.duration import Duration
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger

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

    >> llm_response      string    LLM响应文本
    """

    def __init__(self, active_questioning_service='/active_questioning/trigger_question',
                 llm_response_topic='/llm_response',
                 timeout=10.0):
        """初始化状态"""
        super(ActiveQuestioningState, self).__init__(
            outcomes=['question_asked', 'service_unavailable', 'timeout'],
            output_keys=['llm_response']
        )
        
        # 存储参数
        self._active_questioning_service = active_questioning_service
        self._llm_response_topic = llm_response_topic
        self._timeout = timeout
        
        # 状态标志
        self._question_asked = False
        self._llm_response = ""
        
        # 初始化代理
        ProxyServiceCaller.initialize(ActiveQuestioningState._node)
        ProxySubscriberCached.initialize(ActiveQuestioningState._node)
        
        # 创建服务调用者
        self._trigger_question_client = ProxyServiceCaller()
        # 注册服务客户端 - 使用Trigger而不是Empty，因为大多数主动发问服务使用Trigger
        self._trigger_question_client.setup_service(self._active_questioning_service, Trigger)
        
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
        
        # 检查是否收到LLM响应
        if self._llm_response_sub.has_msg(self._llm_response_topic):
            msg = self._llm_response_sub.get_last_msg(self._llm_response_topic)
            self._llm_response_sub.remove_last_msg(self._llm_response_topic)
            
            self._llm_response = msg.data
            Logger.loginfo(f'收到LLM响应: {self._llm_response}')
            userdata.llm_response = self._llm_response
            return 'question_asked'
        
        # 继续等待LLM响应
        return None

    def on_enter(self, userdata):
        """
        当状态被激活时调用
        """
        self._start_time = self._node.get_clock().now()
        
        # 重置状态标志
        self._question_asked = False
        self._llm_response = ""
        
        # 调用主动发问服务
        try:
            self._trigger_question_client.call(self._active_questioning_service, Trigger.Request())
            Logger.loginfo('已调用主动发问服务')
        except Exception as e:
            Logger.logerr(f'调用主动发问服务失败: {str(e)}')
            return 'service_unavailable'
        
        Logger.loginfo('开始主动发问')

    def on_exit(self, userdata):
        """
        当状态退出时调用
        """
        Logger.loginfo('退出主动发问状态')
