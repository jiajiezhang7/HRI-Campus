#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
语音合成FlexBE状态
"""

import rclpy
from rclpy.duration import Duration
from std_msgs.msg import String, Empty
from audio_common_msgs.msg import AudioData

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached, ProxyPublisher

class SpeechGenerationState(EventState):
    """
    封装语音合成功能的状态。

    该状态接收LLM响应，并将合成的语音发布到指定话题。

    -- llm_response_topic string    LLM响应话题名称
    -- audio_generated_topic string 合成音频话题名称
    -- audio_complete_topic string  音频播放完成话题名称
    -- timeout           float     超时时间（秒）

    <= generated         合成成功
    <= not_generated     未合成
    <= timeout           超时
    """

    def __init__(self, llm_response_topic='/llm_response',
                 audio_generated_topic='/audio_generated',
                 audio_complete_topic='/audio_playback_complete',
                 timeout=30.0):
        """初始化状态"""
        super(SpeechGenerationState, self).__init__(
            outcomes=['generated', 'not_generated', 'timeout'],
            input_keys=['llm_response']
        )
        
        # 存储参数
        self._llm_response_topic = llm_response_topic
        self._audio_generated_topic = audio_generated_topic
        self._audio_complete_topic = audio_complete_topic
        self._timeout = timeout
        
        # 状态标志
        self._speech_generated = False
        self._playback_complete = False
        
        # 初始化代理
        ProxySubscriberCached.initialize(SpeechGenerationState._node)
        ProxyPublisher.initialize(SpeechGenerationState._node)
        
        # 创建订阅者
        self._audio_complete_sub = ProxySubscriberCached()
        self._audio_complete_sub.subscribe(audio_complete_topic, Empty)
        
        # 创建发布者
        self._llm_response_pub = ProxyPublisher()
        self._llm_response_pub.create_publisher(llm_response_topic, String)
        
        # 创建音频生成发布者
        self._audio_generated_pub = ProxyPublisher()
        self._audio_generated_pub.create_publisher(audio_generated_topic, AudioData)

    def execute(self, userdata):
        """
        执行状态逻辑
        
        该方法会被周期性调用，直到返回一个结果
        """
        # 检查超时
        if (self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds) > (self._timeout * 1e9):
            return 'timeout'
        
        # 检查是否收到音频播放完成事件
        if self._audio_complete_sub.has_msg(self._audio_complete_topic):
            self._audio_complete_sub.remove_last_msg(self._audio_complete_topic)
            Logger.loginfo('音频播放完成')
            self._playback_complete = True
            return 'generated'
        
        # 继续等待音频播放完成
        return None

    def on_enter(self, userdata):
        """
        当状态被激活时调用
        """
        self._start_time = self._node.get_clock().now()
        
        # 重置状态标志
        self._speech_generated = False
        self._playback_complete = False
        
        # 发布LLM响应
        if hasattr(userdata, 'llm_response') and userdata.llm_response:
            msg = String()
            msg.data = userdata.llm_response
            self._llm_response_pub.publish(self._llm_response_topic, msg)
            Logger.loginfo(f'发布LLM响应: {userdata.llm_response}')
            
            # 发布空的音频数据消息到/audio_generated话题，触发TTS状态发布节点
            audio_msg = AudioData()
            self._audio_generated_pub.publish(self._audio_generated_topic, audio_msg)
            Logger.loginfo('发布音频生成消息，触发动画显示')
            
            self._speech_generated = True
        
        Logger.loginfo('开始语音合成')

    def on_exit(self, userdata):
        """
        当状态退出时调用
        """
        Logger.loginfo('退出语音合成状态')
