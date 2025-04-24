#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
通用对话系统行为状态机
"""

import rclpy
from flexbe_core import Behavior, Logger, Autonomy
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.core.operatable_state_machine import OperatableStateMachine
from flexbe_core import ConcurrencyContainer, PriorityContainer
from flexbe_states.log_state import LogState
from flexbe_states.operator_decision_state import OperatorDecisionState

# 导入自定义状态
from general_dialogue_flexbe_states.animation_display_state import AnimationDisplayState
from general_dialogue_flexbe_states.camera_system_state import CameraSystemState
from general_dialogue_flexbe_states.speech_recognition_state import SpeechRecognitionState
from general_dialogue_flexbe_states.llm_processing_state import LLMProcessingState
from general_dialogue_flexbe_states.speech_generation_state import SpeechGenerationState
from general_dialogue_flexbe_states.active_questioning_state import ActiveQuestioningState
from general_dialogue_flexbe_states.general_dialogue_state import GeneralDialogueState
from general_dialogue_flexbe_states.wait_for_face_state import WaitForFaceState

class GeneralDialogueBehaviorSM(Behavior):
    """
    通用对话系统行为状态机

    该行为封装了通用对话系统的完整功能，包括人脸检测、语音交互和动画显示。
    """

    def __init__(self, node):
        super(GeneralDialogueBehaviorSM, self).__init__()
        self.name = 'General Dialogue Behavior'
        self.node = node

        # 参数
        self.add_parameter('face_angle_topic', '/face_angle')
        self.add_parameter('llm_response_topic', '/llm_response')
        self.add_parameter('audio_complete_topic', '/audio_playback_complete')
        self.add_parameter('continue_detection_topic', '/continue_detection')
        self.add_parameter('active_questioning_service', '/active_questioning/trigger_question')
        self.add_parameter('interaction_timeout', 300.0)
        
        # 存储参数值
        self._face_angle_topic = '/face_angle'
        self._llm_response_topic = '/llm_response'
        self._audio_complete_topic = '/audio_playback_complete'
        self._continue_detection_topic = '/continue_detection'
        self._active_questioning_service = '/active_questioning/trigger_question'
        self._interaction_timeout = 300.0
        
        # 初始化ROS组件
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        
        # 初始化状态
        AnimationDisplayState.initialize_ros(node)
        CameraSystemState.initialize_ros(node)
        SpeechRecognitionState.initialize_ros(node)
        LLMProcessingState.initialize_ros(node)
        SpeechGenerationState.initialize_ros(node)
        ActiveQuestioningState.initialize_ros(node)
        GeneralDialogueState.initialize_ros(node)
        LogState.initialize_ros(node)
        OperatorDecisionState.initialize_ros(node)

    def create(self):
        # 使用参数默认值或从配置获取的值
        face_angle_topic = self._face_angle_topic
        llm_response_topic = self._llm_response_topic
        audio_complete_topic = self._audio_complete_topic
        continue_detection_topic = self._continue_detection_topic
        active_questioning_service = self._active_questioning_service
        interaction_timeout = self._interaction_timeout

        # 创建状态机
        sm = OperatableStateMachine(outcomes=['finished', 'failed'])

        # 创建详细交互子状态机
        sm_detailed_interaction = self.create_detailed_interaction_sm(
            face_angle_topic=face_angle_topic,
            llm_response_topic=llm_response_topic,
            audio_complete_topic=audio_complete_topic,
            continue_detection_topic=continue_detection_topic,
            active_questioning_service=active_questioning_service,
            interaction_timeout=interaction_timeout
        )

        # 主状态机
        with sm:
            # 初始化日志
            sm.add('InitLog',
                  LogState(text="启动通用对话系统", severity=Logger.REPORT_HINT),
                  transitions={'done': 'InteractionMode'},
                  autonomy={'done': Autonomy.Off})

            # 操作员选择交互模式
            sm.add('InteractionMode',
                  OperatorDecisionState(outcomes=["Autonomous", "Detailed", "Quit"],
                                       hint="选择交互模式", suggestion="Autonomous"),
                  transitions={'Autonomous': 'FullAutonomyInteraction',
                              'Detailed': 'DetailedInteraction',
                              'Quit': 'finished'},
                  autonomy={'Autonomous': Autonomy.Off, 'Detailed': Autonomy.Off, 'Quit': Autonomy.Off})

            # 全自主交互模式
            sm.add('FullAutonomyInteraction',
                  GeneralDialogueState(
                      face_angle_topic=face_angle_topic,
                      llm_response_topic=llm_response_topic,
                      audio_complete_topic=audio_complete_topic,
                      continue_detection_topic=continue_detection_topic,
                      active_questioning_service=active_questioning_service,
                      timeout=interaction_timeout),
                  transitions={'interaction_complete': 'InteractionCompleteLog',
                             'interaction_failed': 'InteractionFailedLog',
                             'timeout': 'TimeoutLog'},
                  autonomy={'interaction_complete': Autonomy.Off, 'interaction_failed': Autonomy.Off, 'timeout': Autonomy.Off})

            # 详细交互模式
            sm.add('DetailedInteraction',
                  sm_detailed_interaction,
                  transitions={'interaction_complete': 'InteractionCompleteLog',
                             'interaction_failed': 'InteractionFailedLog',
                             'timeout': 'TimeoutLog'},
                  autonomy={'interaction_complete': Autonomy.Off, 'interaction_failed': Autonomy.Off, 'timeout': Autonomy.Off})

            # 交互完成日志
            sm.add('InteractionCompleteLog',
                  LogState(text="通用对话系统交互成功完成", severity=Logger.REPORT_HINT),
                  transitions={'done': 'finished'},
                  autonomy={'done': Autonomy.Off})

            # 交互失败日志
            sm.add('InteractionFailedLog',
                  LogState(text="通用对话系统交互失败", severity=Logger.REPORT_HINT),
                  transitions={'done': 'InteractionMode'},
                  autonomy={'done': Autonomy.Off})

            # 超时日志
            sm.add('TimeoutLog',
                  LogState(text="通用对话系统交互超时", severity=Logger.REPORT_HINT),
                  transitions={'done': 'InteractionMode'},
                  autonomy={'done': Autonomy.Off})

        return sm

    def create_detailed_interaction_sm(self, face_angle_topic, llm_response_topic, 
                                      audio_complete_topic, continue_detection_topic,
                                      active_questioning_service, interaction_timeout):
        """
        创建详细交互子状态机
        """
        
        # 创建子状态机
        sm_detailed = OperatableStateMachine(outcomes=['interaction_complete', 'interaction_failed', 'timeout'])
        
        with sm_detailed:
            # 初始化动画显示系统
            sm_detailed.add('InitAnimationDisplay',
                                      AnimationDisplayState(timeout=10.0),
                                      transitions={'initialized': 'InitCameraSystem',
                                                 'failed': 'interaction_failed',
                                                 'timeout': 'timeout'},
                                      autonomy={'initialized': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off})
            
            # 初始化摄像头系统
            sm_detailed.add('InitCameraSystem',
                                      CameraSystemState(
                                          face_angle_topic=face_angle_topic,
                                          continue_detection_topic=continue_detection_topic,
                                          timeout=10.0),
                                      transitions={'initialized': 'WaitForHuman',
                                                 'failed': 'interaction_failed',
                                                 'timeout': 'timeout'},
                                      autonomy={'initialized': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off})
            
            # 等待检测到人（真正等待人脸检测topic）
            sm_detailed.add('WaitForHuman',
                           WaitForFaceState(face_angle_topic=face_angle_topic, timeout=30.0),
                           transitions={'detected': 'ActiveQuestioning', 'timeout': 'timeout'},
                           autonomy={'detected': Autonomy.Off, 'timeout': Autonomy.Off})
            
            # 主动发问
            sm_detailed.add('ActiveQuestioning',
                                      ActiveQuestioningState(
                                          active_questioning_service=active_questioning_service,
                                          llm_response_topic=llm_response_topic,
                                          timeout=10.0),
                                      transitions={'question_asked': 'SpeechGeneration',
                                                 'service_unavailable': 'interaction_failed',
                                                 'timeout': 'timeout'},
                                      autonomy={'question_asked': Autonomy.Off, 'service_unavailable': Autonomy.Off, 'timeout': Autonomy.Off},
                                      remapping={'llm_response': 'llm_response'})
            
            # 语音合成
            sm_detailed.add('SpeechGeneration',
                                      SpeechGenerationState(
                                          llm_response_topic=llm_response_topic,
                                          audio_complete_topic=audio_complete_topic,
                                          timeout=30.0),
                                      transitions={'generated': 'WaitForUserResponse',
                                                 'not_generated': 'interaction_failed',
                                                 'timeout': 'timeout'},
                                      autonomy={'generated': Autonomy.Off, 'not_generated': Autonomy.Off, 'timeout': Autonomy.Off},
                                      remapping={'llm_response': 'llm_response'})
            
            # 等待用户响应
            sm_detailed.add('WaitForUserResponse',
                                      LogState(text="等待用户响应...", severity=Logger.REPORT_HINT),
                                      transitions={'done': 'SpeechRecognition'},
                                      autonomy={'done': Autonomy.Off})
            
            # 语音识别
            sm_detailed.add('SpeechRecognition',
                                      SpeechRecognitionState(timeout=30.0),
                                      transitions={'recognized': 'LLMProcessing',
                                                 'not_recognized': 'WaitForUserResponse',
                                                 'timeout': 'timeout'},
                                      autonomy={'recognized': Autonomy.Off, 'not_recognized': Autonomy.Off, 'timeout': Autonomy.Off},
                                      remapping={'recognized_text': 'recognized_text'})
            
            # LLM处理
            sm_detailed.add('LLMProcessing',
                                      LLMProcessingState(timeout=30.0),
                                      transitions={'processed': 'ResponseSpeechGeneration',
                                                 'not_processed': 'interaction_failed',
                                                 'timeout': 'timeout'},
                                      autonomy={'processed': Autonomy.Off, 'not_processed': Autonomy.Off, 'timeout': Autonomy.Off},
                                      remapping={'recognized_text': 'recognized_text',
                                               'llm_response': 'llm_response'})
            
            # 响应语音合成
            sm_detailed.add('ResponseSpeechGeneration',
                                      SpeechGenerationState(
                                          llm_response_topic=llm_response_topic,
                                          audio_complete_topic=audio_complete_topic,
                                          timeout=30.0),
                                      transitions={'generated': 'ContinueInteraction',
                                                 'not_generated': 'interaction_failed',
                                                 'timeout': 'timeout'},
                                      autonomy={'generated': Autonomy.Off, 'not_generated': Autonomy.Off, 'timeout': Autonomy.Off},
                                      remapping={'llm_response': 'llm_response'})
            
            # 继续交互决策
            sm_detailed.add('ContinueInteraction',
                                      OperatorDecisionState(outcomes=["Continue", "Complete"],
                                                           hint="是否继续交互?", suggestion="Continue"),
                                      transitions={'Continue': 'WaitForUserResponse',
                                                  'Complete': 'interaction_complete'},
                                      autonomy={'Continue': Autonomy.Off, 'Complete': Autonomy.Off})
        
        return sm_detailed
        
    def on_param(self, name, value):
        """处理参数更新"""
        Logger.loginfo(f'{name} = {value}')
        
        # 更新内部参数存储
        if name == 'face_angle_topic':
            self._face_angle_topic = value
        elif name == 'llm_response_topic':
            self._llm_response_topic = value
        elif name == 'audio_complete_topic':
            self._audio_complete_topic = value
        elif name == 'continue_detection_topic':
            self._continue_detection_topic = value
        elif name == 'active_questioning_service':
            self._active_questioning_service = value
        elif name == 'interaction_timeout':
            self._interaction_timeout = value
            
        return True


def main():
    """主函数，用于直接运行行为"""
    # 初始化ROS
    rclpy.init()
    
    # 创建节点
    node = rclpy.create_node('general_dialogue_behavior')
    
    # 创建行为
    behavior = GeneralDialogueBehaviorSM(node)
    
    # 执行行为
    try:
        behavior.execute()
    except Exception as e:
        Logger.logerr(f'行为执行失败: {e}')
    finally:
        # 关闭节点
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
