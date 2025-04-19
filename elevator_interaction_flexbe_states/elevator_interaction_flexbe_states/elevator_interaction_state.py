#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
封装电梯交互系统的FlexBE状态
"""

import rclpy
from rclpy.duration import Duration
from std_msgs.msg import String, Float32, Bool, Empty
from std_srvs.srv import Empty as EmptyService

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached, ProxyServiceCaller

class ElevatorInteractionState(EventState):
    """
    封装电梯交互系统的状态。

    该状态监控人脸检测、语音交互和音频播放，协调整个电梯交互过程。

    -- cmd_vel_topic     string    机器人速度命令的话题名称
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

    def __init__(self, cmd_vel_topic='/cmd_vel', 
                 face_angle_topic='/face_angle',
                 llm_response_topic='/llm_response',
                 audio_complete_topic='/audio_playback_complete',
                 continue_detection_topic='/continue_detection',
                 active_questioning_service='/active_questioning/trigger_question',
                 timeout=300.0):
        """初始化状态"""
        super(ElevatorInteractionState, self).__init__(
            outcomes=['interaction_complete', 'interaction_failed', 'timeout']
        )
        
        # 存储参数
        self._cmd_vel_topic = cmd_vel_topic
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
        ProxyPublisher.initialize(ElevatorInteractionState._node)
        ProxySubscriberCached.initialize(ElevatorInteractionState._node)
        ProxyServiceCaller.initialize(ElevatorInteractionState._node)
        
        # 创建发布者
        self._detection_pub = ProxyPublisher()
        self._detection_pub.createPublisher(continue_detection_topic, Bool)
        
        # 创建订阅者
        self._face_angle_sub = ProxySubscriberCached()
        self._face_angle_sub.subscribe(face_angle_topic, Float32)
        
        self._llm_response_sub = ProxySubscriberCached()
        self._llm_response_sub.subscribe(llm_response_topic, String)
        
        self._playback_complete_sub = ProxySubscriberCached()
        self._playback_complete_sub.subscribe(audio_complete_topic, Empty)
        
        # 创建服务客户端
        self._trigger_question_client = ProxyServiceCaller()
        self._trigger_question_client.create_client(active_questioning_service, EmptyService)

    def execute(self, userdata):
        """
        执行状态逻辑
        
        该方法会被周期性调用，直到返回一个结果
        """
        # 检查超时
        if (self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds) > (self._timeout * 1e9):
            return 'timeout'
            
        # 检查交互是否完成
        if self._interaction_complete:
            return 'interaction_complete'
            
        # 处理人脸角度消息
        if self._face_angle_sub.has_msg(self._face_angle_topic) and not self._questioning_active:
            msg = self._face_angle_sub.get_last_msg(self._face_angle_topic)
            self._face_angle_sub.remove_last_msg(self._face_angle_topic)
            
            if not self._questioning_active and not self._interaction_complete:
                if not (self._waiting_for_playback and self._received_playback_event):
                    face_angle = msg.data
                    Logger.loginfo(f'检测到人脸，角度: {face_angle}°')
                    self._trigger_questioning()
        
        # 处理LLM响应消息
        if self._llm_response_sub.has_msg(self._llm_response_topic) and self._questioning_active:
            msg = self._llm_response_sub.get_last_msg(self._llm_response_topic)
            self._llm_response_sub.remove_last_msg(self._llm_response_topic)
            
            response_text = msg.data
            Logger.loginfo(f'收到LLM响应: {response_text}')
            
            positive_keywords = ["good", "谢谢"]
            negative_keywords = ["jerk", "另找人"]
            
            if any(keyword in response_text for keyword in positive_keywords):
                Logger.loginfo('检测到积极响应，停止交互')
                self._interaction_complete = True
            elif any(keyword in response_text for keyword in negative_keywords):
                Logger.loginfo('检测到消极响应，等待音频播放完成后继续寻找人')
                self._waiting_for_playback = True
                self._pending_reset = True
        
        # 处理音频播放完成事件
        if self._playback_complete_sub.has_msg(self._audio_complete_topic):
            self._playback_complete_sub.remove_last_msg(self._audio_complete_topic)
            Logger.loginfo('音频播放完成')
            self._received_playback_event = True
            
            if self._waiting_for_playback and self._pending_reset:
                Logger.loginfo('重置状态，继续寻找人')
                self._waiting_for_playback = False
                self._pending_reset = False
                self._questioning_active = False
        
        # 发布继续检测标志
        msg = Bool()
        if not self._received_playback_event:
            msg.data = not self._interaction_complete
        else:
            msg.data = not (self._interaction_complete or self._waiting_for_playback)
        self._detection_pub.publish(self._continue_detection_topic, msg)
        
        # 如果交互尚未完成，继续执行
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
        
        # 发布初始状态
        msg = Bool()
        msg.data = True
        self._detection_pub.publish(self._continue_detection_topic, msg)
        Logger.loginfo('发布初始检测状态：允许检测')

    def on_exit(self, userdata):
        """
        当状态退出时调用
        """
        # 停止检测
        msg = Bool()
        msg.data = False
        self._detection_pub.publish(self._continue_detection_topic, msg)
        Logger.loginfo('退出状态，停止检测')

    def _trigger_questioning(self):
        """
        触发主动发问服务
        """
        if not self._trigger_question_client.is_available(self._active_questioning_service):
            Logger.logwarn('active_questioning服务不可用')
            return
        
        Logger.loginfo('触发主动发问')
        self._questioning_active = True
        
        request = EmptyService.Request()
        self._trigger_question_client.call_async(self._active_questioning_service, request,
                                               lambda result: self._questioning_callback(result))
    
    def _questioning_callback(self, result):
        """
        主动发问服务调用完成的回调
        """
        try:
            Logger.loginfo('主动发问已触发')
        except Exception as e:
            Logger.logerr(f'触发主动发问时出错: {str(e)}')
            self._questioning_active = False
