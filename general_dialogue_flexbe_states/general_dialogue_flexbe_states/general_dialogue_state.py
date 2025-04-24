#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
通用对话系统FlexBE状态
"""

import rclpy
from rclpy.duration import Duration
from std_msgs.msg import String, Float32, Bool, Empty
from std_srvs.srv import Empty as EmptyService

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached, ProxyPublisher, ProxyServiceCaller

class GeneralDialogueState(EventState):
    """
    封装通用对话系统的状态。

    该状态监控人脸检测、语音交互和音频播放，协调整个通用对话系统交互过程。

    -- face_angle_topic  string    人脸角度话题名称
    -- llm_response_topic string    LLM响应话题名称
    -- audio_complete_topic string  音频播放完成话题名称
    -- continue_detection_topic string 继续检测标志话题名称
    -- active_questioning_service string 主动发问服务名称
    -- timeout           float     超时时间（秒）

    <= interaction_complete    交互成功完成
    <= interaction_failed      交互失败
    <= timeout                 超时
    """

    def __init__(self, face_angle_topic='/face_angle',
                 llm_response_topic='/llm_response',
                 audio_complete_topic='/audio_playback_complete',
                 continue_detection_topic='/continue_detection',
                 active_questioning_service='/active_questioning/trigger_question',
                 timeout=300.0):
        """初始化状态"""
        super(GeneralDialogueState, self).__init__(
            outcomes=['interaction_complete', 'interaction_failed', 'timeout']
        )

        # 存储参数
        self._face_angle_topic = face_angle_topic
        self._llm_response_topic = llm_response_topic
        self._audio_complete_topic = audio_complete_topic
        self._continue_detection_topic = continue_detection_topic
        self._active_questioning_service = active_questioning_service
        self._timeout = timeout

        # 状态标志
        self._questioning_active = False
        self._interaction_complete = False
        self._waiting_for_playback = False
        self._pending_reset = False
        self._received_playback_event = False

        # 初始化代理
        ProxySubscriberCached.initialize(GeneralDialogueState._node)
        ProxyPublisher.initialize(GeneralDialogueState._node)
        ProxyServiceCaller.initialize(GeneralDialogueState._node)

        # 创建订阅者
        self._face_angle_sub = ProxySubscriberCached()
        self._face_angle_sub.subscribe(face_angle_topic, Float32)

        self._llm_response_sub = ProxySubscriberCached()
        self._llm_response_sub.subscribe(llm_response_topic, String)

        self._playback_complete_sub = ProxySubscriberCached()
        self._playback_complete_sub.subscribe(audio_complete_topic, Empty)

        # 创建发布者
        self._detection_pub = ProxyPublisher()
        # 注册发布者主题
        self._detection_pub.create_publisher(continue_detection_topic, Bool)

        # 创建服务调用者
        self._trigger_question_client = ProxyServiceCaller()
        # 注册服务客户端
        self._trigger_question_client.setup_service(active_questioning_service, EmptyService)

    def execute(self, userdata):
        """
        执行状态逻辑

        该方法会被周期性调用，直到返回一个结果
        """
        # 检查超时
        if (self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds) > (self._timeout * 1e9):
            return 'timeout'

        # 检查是否收到音频播放完成事件
        if self._playback_complete_sub.has_msg(self._audio_complete_topic):
            self._playback_complete_sub.remove_last_msg(self._audio_complete_topic)
            Logger.loginfo('收到音频播放完成事件')
            self._received_playback_event = True

            if self._waiting_for_playback:
                self._waiting_for_playback = False

                # 如果需要重置交互
                if self._pending_reset:
                    self._pending_reset = False
                    self._questioning_active = False

                    # 发布继续检测标志
                    msg = Bool()
                    msg.data = True
                    self._detection_pub.publish(self._continue_detection_topic, msg)
                    Logger.loginfo('重置交互状态，允许继续检测')

        # 检查是否收到LLM响应
        if self._llm_response_sub.has_msg(self._llm_response_topic):
            msg = self._llm_response_sub.get_last_msg(self._llm_response_topic)
            self._llm_response_sub.remove_last_msg(self._llm_response_topic)

            response_text = msg.data
            Logger.loginfo(f'收到LLM响应: {response_text}')

            # 标记等待音频播放完成
            self._waiting_for_playback = True
            self._received_playback_event = False

            # 如果是主动发问，则在播放完成后重置状态
            if self._questioning_active:
                self._pending_reset = True

        # 检查是否检测到人脸
        if self._face_angle_sub.has_msg(self._face_angle_topic) and not self._questioning_active and not self._interaction_complete:
            msg = self._face_angle_sub.get_last_msg(self._face_angle_topic)
            self._face_angle_sub.remove_last_msg(self._face_angle_topic)

            face_angle = msg.data
            Logger.loginfo(f'检测到人脸，角度: {face_angle}°')

            # 设置状态标志，防止重复触发
            self._questioning_active = True

            # 发布停止检测标志
            stop_msg = Bool()
            stop_msg.data = False
            self._detection_pub.publish(self._continue_detection_topic, stop_msg)

            # 触发主动发问
            self._trigger_questioning()

        # 继续执行
        return None

    def on_enter(self, userdata):
        """
        当状态被激活时调用
        """
        self._start_time = self._node.get_clock().now()

        # 重置状态标志
        self._questioning_active = False
        self._interaction_complete = False
        self._waiting_for_playback = False
        self._pending_reset = False
        self._received_playback_event = False

        # 发布初始状态，确保开始时能够检测人脸
        msg = Bool()
        msg.data = True
        self._detection_pub.publish(self._continue_detection_topic, msg)
        Logger.loginfo('发布初始检测状态：允许检测')

        Logger.loginfo('通用对话系统状态已激活')

    def on_exit(self, userdata):
        """
        当状态退出时调用
        """
        Logger.loginfo('退出通用对话系统状态')

    def _trigger_questioning(self):
        """
        触发主动发问
        """
        if self._questioning_active:
            Logger.loginfo('主动发问已激活，忽略重复触发')
            return

        Logger.loginfo('触发主动发问')

        # 调用主动发问服务
        try:
            # 创建空请求对象
            request = EmptyService.Request()
            self._trigger_question_client.call_async(self._active_questioning_service, request)
            Logger.loginfo('已调用主动发问服务')
        except Exception as e:
            Logger.logerr(f'调用主动发问服务失败: {str(e)}')
            self._questioning_active = False

            # 恢复检测
            msg = Bool()
            msg.data = True
            self._detection_pub.publish(self._continue_detection_topic, msg)
