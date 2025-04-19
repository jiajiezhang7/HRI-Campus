#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
语音识别FlexBE状态
"""

import rclpy
from rclpy.duration import Duration
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached, ProxyPublisher

class SpeechRecognitionState(EventState):
    """
    封装语音识别功能的状态。

    该状态监听音频数据，并将识别结果发布到指定话题。

    -- audio_topic       string    音频数据话题名称
    -- speech_text_topic string    语音识别结果话题名称
    -- timeout           float     超时时间（秒）

    <= recognized        识别成功
    <= not_recognized    未识别到语音
    <= timeout           超时
    """

    def __init__(self, audio_topic='/audio/audio',
                 speech_text_topic='/speech_to_text',
                 timeout=30.0):
        """初始化状态"""
        super(SpeechRecognitionState, self).__init__(
            outcomes=['recognized', 'not_recognized', 'timeout'],
            output_keys=['recognized_text']
        )
        
        # 存储参数
        self._audio_topic = audio_topic
        self._speech_text_topic = speech_text_topic
        self._timeout = timeout
        
        # 状态标志
        self._speech_recognized = False
        self._recognized_text = ""
        
        # 初始化代理
        ProxySubscriberCached.initialize(SpeechRecognitionState._node)
        ProxyPublisher.initialize(SpeechRecognitionState._node)
        
        # 创建订阅者
        self._audio_sub = ProxySubscriberCached()
        self._audio_sub.subscribe(audio_topic, AudioData)
        
        self._speech_text_sub = ProxySubscriberCached()
        self._speech_text_sub.subscribe(speech_text_topic, String)

    def execute(self, userdata):
        """
        执行状态逻辑
        
        该方法会被周期性调用，直到返回一个结果
        """
        # 检查超时
        if (self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds) > (self._timeout * 1e9):
            return 'timeout'
        
        # 检查是否收到语音识别结果
        if self._speech_text_sub.has_msg(self._speech_text_topic):
            msg = self._speech_text_sub.get_last_msg(self._speech_text_topic)
            self._speech_text_sub.remove_last_msg(self._speech_text_topic)
            
            self._recognized_text = msg.data
            Logger.loginfo(f'识别到语音: {self._recognized_text}')
            self._speech_recognized = True
            userdata.recognized_text = self._recognized_text
            return 'recognized'
        
        # 继续等待语音识别结果
        return None

    def on_enter(self, userdata):
        """
        当状态被激活时调用
        """
        self._start_time = self._node.get_clock().now()
        
        # 重置状态标志
        self._speech_recognized = False
        self._recognized_text = ""
        
        Logger.loginfo('开始语音识别')

    def on_exit(self, userdata):
        """
        当状态退出时调用
        """
        Logger.loginfo('退出语音识别状态')
