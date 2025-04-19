#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
电梯交互行为状态机
"""

from flexbe_core import Behavior, Autonomy
from flexbe_core import OperatableStateMachine, Logger, ConcurrencyContainer, PriorityContainer
from flexbe_states.log_state import LogState
from flexbe_states.operator_decision_state import OperatorDecisionState

# 导入自定义状态
from elevator_interaction_flexbe_states.elevator_interaction_state import ElevatorInteractionState
from elevator_interaction_flexbe_states.human_detection_state import HumanDetectionState
from elevator_interaction_flexbe_states.active_questioning_state import ActiveQuestioningState
from elevator_interaction_flexbe_states.speech_recognition_state import SpeechRecognitionState
from elevator_interaction_flexbe_states.llm_processing_state import LLMProcessingState
from elevator_interaction_flexbe_states.speech_generation_state import SpeechGenerationState

class ElevatorInteractionBehaviorSM(Behavior):
    """
    电梯交互行为状态机

    该行为封装了电梯交互系统的完整功能，包括人体检测、语音交互和动画显示。
    """

    def __init__(self, node):
        super(ElevatorInteractionBehaviorSM, self).__init__()
        self.name = 'Elevator Interaction Behavior'

        # 参数
        self.add_parameter('cmd_vel_topic', '/cmd_vel')
        self.add_parameter('face_angle_topic', '/face_angle')
        self.add_parameter('llm_response_topic', '/llm_response')
        self.add_parameter('audio_complete_topic', '/audio_playback_complete')
        self.add_parameter('continue_detection_topic', '/continue_detection')
        self.add_parameter('active_questioning_service', '/active_questioning/trigger_question')
        self.add_parameter('interaction_timeout', 300.0)

        # 初始化ROS组件
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)

        # 初始化状态
        ElevatorInteractionState.initialize_ros(node)
        HumanDetectionState.initialize_ros(node)
        ActiveQuestioningState.initialize_ros(node)
        SpeechRecognitionState.initialize_ros(node)
        LLMProcessingState.initialize_ros(node)
        SpeechGenerationState.initialize_ros(node)
        LogState.initialize_ros(node)
        OperatorDecisionState.initialize_ros(node)

    def create(self):
        # 获取参数
        cmd_vel_topic = self.cmd_vel_topic
        face_angle_topic = self.face_angle_topic
        llm_response_topic = self.llm_response_topic
        audio_complete_topic = self.audio_complete_topic
        continue_detection_topic = self.continue_detection_topic
        active_questioning_service = self.active_questioning_service
        interaction_timeout = self.interaction_timeout

        # 创建状态机
        sm = OperatableStateMachine(outcomes=['finished', 'failed'])

        # 创建详细交互状态机
        sm_detailed_interaction = OperatableStateMachine(outcomes=['interaction_complete', 'interaction_failed', 'timeout'])

        # 详细交互状态机
        with sm_detailed_interaction:
            # 人体检测
            OperatableStateMachine.add('HumanDetection',
                                      HumanDetectionState(
                                          face_angle_topic=face_angle_topic,
                                          continue_detection_topic=continue_detection_topic,
                                          timeout=60.0),
                                      transitions={'detected': 'ActiveQuestioning',
                                                 'not_detected': 'HumanDetection',
                                                 'timeout': 'timeout'},
                                      autonomy={'detected': Autonomy.Off,
                                               'not_detected': Autonomy.Off,
                                               'timeout': Autonomy.Off})

            # 主动发问
            OperatableStateMachine.add('ActiveQuestioning',
                                      ActiveQuestioningState(
                                          active_questioning_service=active_questioning_service,
                                          llm_response_topic=llm_response_topic,
                                          timeout=10.0),
                                      transitions={'question_asked': 'SpeechGeneration',
                                                 'service_unavailable': 'ServiceUnavailableLog',
                                                 'timeout': 'QuestioningTimeoutLog'},
                                      autonomy={'question_asked': Autonomy.Off,
                                               'service_unavailable': Autonomy.Off,
                                               'timeout': Autonomy.Off})

            # 服务不可用日志
            OperatableStateMachine.add('ServiceUnavailableLog',
                                      LogState(text="主动发问服务不可用", severity=Logger.REPORT_WARN),
                                      transitions={'done': 'interaction_failed'},
                                      autonomy={'done': Autonomy.Off})

            # 发问超时日志
            OperatableStateMachine.add('QuestioningTimeoutLog',
                                      LogState(text="主动发问超时", severity=Logger.REPORT_WARN),
                                      transitions={'done': 'interaction_failed'},
                                      autonomy={'done': Autonomy.Off})

            # 语音合成（问题）
            OperatableStateMachine.add('SpeechGeneration',
                                      SpeechGenerationState(
                                          llm_response_topic=llm_response_topic,
                                          audio_generated_topic='/audio_generated',
                                          audio_complete_topic=audio_complete_topic,
                                          timeout=30.0),
                                      transitions={'generated': 'SpeechRecognition',
                                                 'not_generated': 'SpeechGenerationFailedLog',
                                                 'timeout': 'SpeechGenerationTimeoutLog'},
                                      autonomy={'generated': Autonomy.Off,
                                               'not_generated': Autonomy.Off,
                                               'timeout': Autonomy.Off},
                                      remapping={'llm_response': 'llm_response'})

            # 语音合成失败日志
            OperatableStateMachine.add('SpeechGenerationFailedLog',
                                      LogState(text="语音合成失败", severity=Logger.REPORT_WARN),
                                      transitions={'done': 'interaction_failed'},
                                      autonomy={'done': Autonomy.Off})

            # 语音合成超时日志
            OperatableStateMachine.add('SpeechGenerationTimeoutLog',
                                      LogState(text="语音合成超时", severity=Logger.REPORT_WARN),
                                      transitions={'done': 'interaction_failed'},
                                      autonomy={'done': Autonomy.Off})

            # 语音识别
            OperatableStateMachine.add('SpeechRecognition',
                                      SpeechRecognitionState(
                                          audio_topic='/audio/audio',
                                          speech_text_topic='/speech_to_text',
                                          timeout=30.0),
                                      transitions={'recognized': 'LLMProcessing',
                                                 'not_recognized': 'SpeechRecognitionFailedLog',
                                                 'timeout': 'SpeechRecognitionTimeoutLog'},
                                      autonomy={'recognized': Autonomy.Off,
                                               'not_recognized': Autonomy.Off,
                                               'timeout': Autonomy.Off},
                                      remapping={'recognized_text': 'recognized_text'})

            # 语音识别失败日志
            OperatableStateMachine.add('SpeechRecognitionFailedLog',
                                      LogState(text="语音识别失败", severity=Logger.REPORT_WARN),
                                      transitions={'done': 'interaction_failed'},
                                      autonomy={'done': Autonomy.Off})

            # 语音识别超时日志
            OperatableStateMachine.add('SpeechRecognitionTimeoutLog',
                                      LogState(text="语音识别超时", severity=Logger.REPORT_WARN),
                                      transitions={'done': 'interaction_failed'},
                                      autonomy={'done': Autonomy.Off})

            # LLM处理
            OperatableStateMachine.add('LLMProcessing',
                                      LLMProcessingState(
                                          speech_text_topic='/speech_to_text',
                                          llm_response_topic=llm_response_topic,
                                          timeout=30.0),
                                      transitions={'processed': 'ResponseSpeechGeneration',
                                                 'not_processed': 'LLMProcessingFailedLog',
                                                 'timeout': 'LLMProcessingTimeoutLog'},
                                      autonomy={'processed': Autonomy.Off,
                                               'not_processed': Autonomy.Off,
                                               'timeout': Autonomy.Off},
                                      remapping={'recognized_text': 'recognized_text',
                                                'llm_response': 'llm_response'})

            # LLM处理失败日志
            OperatableStateMachine.add('LLMProcessingFailedLog',
                                      LogState(text="LLM处理失败", severity=Logger.REPORT_WARN),
                                      transitions={'done': 'interaction_failed'},
                                      autonomy={'done': Autonomy.Off})

            # LLM处理超时日志
            OperatableStateMachine.add('LLMProcessingTimeoutLog',
                                      LogState(text="LLM处理超时", severity=Logger.REPORT_WARN),
                                      transitions={'done': 'interaction_failed'},
                                      autonomy={'done': Autonomy.Off})

            # 语音合成（响应）
            OperatableStateMachine.add('ResponseSpeechGeneration',
                                      SpeechGenerationState(
                                          llm_response_topic=llm_response_topic,
                                          audio_generated_topic='/audio_generated',
                                          audio_complete_topic=audio_complete_topic,
                                          timeout=30.0),
                                      transitions={'generated': 'CheckResponse',
                                                 'not_generated': 'ResponseSpeechGenerationFailedLog',
                                                 'timeout': 'ResponseSpeechGenerationTimeoutLog'},
                                      autonomy={'generated': Autonomy.Off,
                                               'not_generated': Autonomy.Off,
                                               'timeout': Autonomy.Off},
                                      remapping={'llm_response': 'llm_response'})

            # 响应语音合成失败日志
            OperatableStateMachine.add('ResponseSpeechGenerationFailedLog',
                                      LogState(text="响应语音合成失败", severity=Logger.REPORT_WARN),
                                      transitions={'done': 'interaction_failed'},
                                      autonomy={'done': Autonomy.Off})

            # 响应语音合成超时日志
            OperatableStateMachine.add('ResponseSpeechGenerationTimeoutLog',
                                      LogState(text="响应语音合成超时", severity=Logger.REPORT_WARN),
                                      transitions={'done': 'interaction_failed'},
                                      autonomy={'done': Autonomy.Off})

            # 检查响应
            OperatableStateMachine.add('CheckResponse',
                                      LogState(text="检查响应", severity=Logger.REPORT_HINT),
                                      transitions={'done': 'interaction_complete'},
                                      autonomy={'done': Autonomy.Off})

        # 主状态机
        with sm:
            # 初始化日志
            OperatableStateMachine.add('InitLog',
                                      LogState(text="启动电梯交互系统", severity=Logger.REPORT_HINT),
                                      transitions={'done': 'InteractionMode'},
                                      autonomy={'done': Autonomy.Off})

            # 操作员选择交互模式
            OperatableStateMachine.add('InteractionMode',
                                      OperatorDecisionState(outcomes=["Autonomous", "Detailed", "Quit"],
                                                           hint="选择交互模式", suggestion="Autonomous"),
                                      transitions={'Autonomous': 'FullAutonomyInteraction',
                                                  'Detailed': 'DetailedInteraction',
                                                  'Quit': 'finished'},
                                      autonomy={'Autonomous': Autonomy.Full,
                                               'Detailed': Autonomy.High,
                                               'Quit': Autonomy.Full})

            # 全自主交互模式
            OperatableStateMachine.add('FullAutonomyInteraction',
                                      ElevatorInteractionState(
                                          cmd_vel_topic=cmd_vel_topic,
                                          face_angle_topic=face_angle_topic,
                                          llm_response_topic=llm_response_topic,
                                          audio_complete_topic=audio_complete_topic,
                                          continue_detection_topic=continue_detection_topic,
                                          active_questioning_service=active_questioning_service,
                                          timeout=interaction_timeout),
                                      transitions={'interaction_complete': 'InteractionCompleteLog',
                                                 'interaction_failed': 'InteractionFailedLog',
                                                 'timeout': 'TimeoutLog'},
                                      autonomy={'interaction_complete': Autonomy.Off,
                                               'interaction_failed': Autonomy.Off,
                                               'timeout': Autonomy.Off})

            # 详细交互模式
            OperatableStateMachine.add('DetailedInteraction',
                                      sm_detailed_interaction,
                                      transitions={'interaction_complete': 'InteractionCompleteLog',
                                                 'interaction_failed': 'InteractionFailedLog',
                                                 'timeout': 'TimeoutLog'},
                                      autonomy={'interaction_complete': Autonomy.Inherit,
                                               'interaction_failed': Autonomy.Inherit,
                                               'timeout': Autonomy.Inherit})

            # 交互完成日志
            OperatableStateMachine.add('InteractionCompleteLog',
                                      LogState(text="电梯交互成功完成", severity=Logger.REPORT_HINT),
                                      transitions={'done': 'InteractionMode'},
                                      autonomy={'done': Autonomy.Off})

            # 交互失败日志
            OperatableStateMachine.add('InteractionFailedLog',
                                      LogState(text="电梯交互失败", severity=Logger.REPORT_HINT),
                                      transitions={'done': 'InteractionMode'},
                                      autonomy={'done': Autonomy.Off})

            # 超时日志
            OperatableStateMachine.add('TimeoutLog',
                                      LogState(text="电梯交互超时", severity=Logger.REPORT_HINT),
                                      transitions={'done': 'InteractionMode'},
                                      autonomy={'done': Autonomy.Off})

        return sm


def main():
    """主函数，用于直接运行行为"""
    import rclpy
    from flexbe_core import Logger

    rclpy.init()
    node = rclpy.create_node('elevator_interaction_behavior_node')
    Logger.initialize(node)

    # 创建行为实例
    behavior = ElevatorInteractionBehaviorSM(node)

    # 创建状态机
    behavior.create()

    node.get_logger().info('行为已创建，可以通过FlexBE启动')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == '__main__':
    main()
