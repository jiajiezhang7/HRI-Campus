#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
大语言模型处理FlexBE状态
"""

import rclpy
from rclpy.duration import Duration
from std_msgs.msg import String

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached, ProxyPublisher

class LLMProcessingState(EventState):
    """
    封装大语言模型处理功能的状态。

    该状态接收语音识别结果，并将LLM处理结果发布到指定话题。

    -- speech_text_topic string    语音识别结果话题名称
    -- llm_response_topic string    LLM响应话题名称
    -- timeout           float     超时时间（秒）

    <= processed         处理成功
    <= not_processed     未处理
    <= timeout           超时

    >> llm_response      string    LLM响应文本
    """

    def __init__(self, speech_text_topic='/speech_to_text',
                 llm_response_topic='/llm_response',
                 timeout=30.0):
        """初始化状态"""
        super(LLMProcessingState, self).__init__(
            outcomes=['processed', 'not_processed', 'timeout'],
            input_keys=['recognized_text'],
            output_keys=['llm_response']
        )
        
        # 存储参数
        self._speech_text_topic = speech_text_topic
        self._llm_response_topic = llm_response_topic
        self._timeout = timeout
        
        # 状态标志
        self._llm_processed = False
        self._llm_response = ""
        
        # 初始化代理
        ProxySubscriberCached.initialize(LLMProcessingState._node)
        ProxyPublisher.initialize(LLMProcessingState._node)
        
        # 创建订阅者
        self._llm_response_sub = ProxySubscriberCached()
        self._llm_response_sub.subscribe(llm_response_topic, String)
        
        # 创建发布者
        self._speech_text_pub = ProxyPublisher()

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
            self._llm_processed = True
            userdata.llm_response = self._llm_response
            return 'processed'
        
        # 继续等待LLM响应
        return None

    def on_enter(self, userdata):
        """
        当状态被激活时调用
        """
        self._start_time = self._node.get_clock().now()
        
        # 重置状态标志
        self._llm_processed = False
        self._llm_response = ""
        
        # 发布语音识别结果
        if hasattr(userdata, 'recognized_text') and userdata.recognized_text:
            msg = String()
            msg.data = userdata.recognized_text
            self._speech_text_pub.publish(self._speech_text_topic, msg)
            Logger.loginfo(f'发布语音识别结果: {userdata.recognized_text}')
        
        Logger.loginfo('开始LLM处理')

    def on_exit(self, userdata):
        """
        当状态退出时调用
        """
        Logger.loginfo('退出LLM处理状态')
