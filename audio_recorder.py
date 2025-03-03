#!/usr/bin/env python3
# 订阅audio_generated话题，保存为wav音频文件，用于debug
import rclpy
from rclpy.node import Node
from audio_common_msgs.msg import AudioData
import time
import os
import wave
import numpy as np
import struct

class AudioRecorder(Node):
    def __init__(self):
        super().__init__('audio_recorder')
        
        # 声明参数
        self.declare_parameter('channels', 2)  # 声道数
        self.declare_parameter('sample_width', 2)  # 采样宽度（字节）
        self.declare_parameter('sample_rate', 16000)  # 采样率
        self.declare_parameter('max_duration', 60)  # 最大录音时长（秒）
        
        # 获取参数
        self.channels = self.get_parameter('channels').value
        self.sample_width = self.get_parameter('sample_width').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.max_duration = self.get_parameter('max_duration').value
        
        # 创建订阅
        self.subscription = self.create_subscription(
            AudioData,
            '/audio/audio',
            self.listener_callback,
            10)
        self.get_logger().info('音频记录器已启动，正在监听 /audio/audio 话题，用于检测滤波效果')
        self.get_logger().info(f'音频格式: {self.channels}声道, {self.sample_width*8}位, {self.sample_rate}Hz')
        
        # 创建保存目录
        self.save_dir = os.path.expanduser('~/audio_recordings')
        os.makedirs(self.save_dir, exist_ok=True)
        
        # 初始化记录状态
        self.is_recording = False
        self.current_file = None
        self.current_wave = None
        self.start_time = None
        self.data_received = 0
        self.audio_buffer = bytearray()
        
        # 创建定时器，每10秒报告一次状态
        self.timer = self.create_timer(10.0, self.report_status)
        
        # 创建定时器，每隔max_duration秒创建一个新文件
        self.file_timer = self.create_timer(self.max_duration, self.rotate_file)
        
        # 创建原始数据文件，用于调试
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        self.raw_file = open(os.path.join(self.save_dir, f'raw_audio_{timestamp}.bin'), 'wb')
        self.get_logger().info(f'创建原始数据文件: {self.raw_file.name}')
    
    def listener_callback(self, msg):
        # 保存原始数据用于调试
        self.raw_file.write(bytes(msg.data))
        self.raw_file.flush()
        
        # 第一次收到数据时开始记录
        if not self.is_recording:
            self.start_recording()
        
        # 记录数据
        if self.current_wave:
            # 检查数据是否有效
            if len(msg.data) > 0:
                try:
                    # 将数据添加到缓冲区
                    self.audio_buffer.extend(msg.data)
                    self.data_received += len(msg.data)
                    
                    # 每累积一定量的数据就写入文件
                    if len(self.audio_buffer) >= 32000:  # 约1秒的音频
                        self.current_wave.writeframes(bytes(self.audio_buffer))
                        
                        # 打印一些样本值用于调试
                        if len(self.audio_buffer) >= 20:
                            samples = struct.unpack(f'<{min(10, len(self.audio_buffer)//2)}h', 
                                                   self.audio_buffer[:min(20, len(self.audio_buffer))])
                            # self.get_logger().info(f'样本值: {samples}')
                        
                        self.audio_buffer = bytearray()
                except Exception as e:
                    self.get_logger().error(f'处理音频数据错误: {str(e)}')
    
    def start_recording(self):
        self.close_current_file()
        
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        filepath = os.path.join(self.save_dir, f'audio_{timestamp}.wav')
        
        # 创建WAV文件
        self.current_file = wave.open(filepath, 'wb')
        self.current_file.setnchannels(self.channels)
        self.current_file.setsampwidth(self.sample_width)
        self.current_file.setframerate(self.sample_rate)
        self.current_wave = self.current_file
        
        self.is_recording = True
        self.start_time = time.time()
        self.data_received = 0
        self.audio_buffer = bytearray()
        
        self.get_logger().info(f'开始记录音频到: {filepath}')
        self.get_logger().info(f'音频格式: {self.channels}声道, {self.sample_width*8}位, {self.sample_rate}Hz')
    
    def close_current_file(self):
        if self.current_wave:
            # 写入剩余的缓冲数据
            if len(self.audio_buffer) > 0:
                self.current_wave.writeframes(bytes(self.audio_buffer))
                self.audio_buffer = bytearray()
            
            self.current_wave.close()
            self.current_wave = None
            self.current_file = None
            
            if self.is_recording:
                duration = time.time() - self.start_time
                self.get_logger().info(f'已完成录音，总时长: {duration:.1f}秒')
    
    def rotate_file(self):
        if self.is_recording:
            self.get_logger().info('达到最大录音时长，创建新文件')
            self.start_recording()
    
    def report_status(self):
        if self.is_recording:
            duration = time.time() - self.start_time
            kb_received = self.data_received / 1024
            self.get_logger().info(f'已记录 {duration:.1f} 秒的音频，接收了 {kb_received:.2f} KB 数据')
            
            # 计算音频能量，检测是否有声音
            if len(self.audio_buffer) > 0:
                try:
                    audio_array = np.frombuffer(self.audio_buffer, dtype=np.int16)
                    energy = np.sqrt(np.mean(np.square(audio_array.astype(np.float32))))
                    self.get_logger().info(f'当前音频能量: {energy:.2f}')
                except Exception as e:
                    self.get_logger().error(f'计算音频能量错误: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    recorder = AudioRecorder()
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.close_current_file()
        if hasattr(recorder, 'raw_file') and recorder.raw_file:
            recorder.raw_file.close()
        recorder.get_logger().info('音频记录已停止')
    finally:
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
