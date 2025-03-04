#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2节点，订阅音频数据并播放到系统扬声器
使用标准库和外部命令播放音频，不依赖于GStreamer
"""

import os
import sys
import time
import tempfile
import subprocess
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from audio_common_msgs.msg import AudioData
from std_msgs.msg import Empty


class AudioPlayNode(Node):
    """
    音频播放节点，将接收到的音频数据通过系统扬声器播放出来
    """
    def __init__(self):
        super().__init__('audio_play_node')
        
        # 声明参数
        self.declare_parameter('format', 'wave')  # 音频格式
        self.declare_parameter('sample_rate', 16000)  # 采样率
        self.declare_parameter('channels', 2)  # 声道数
        self.declare_parameter('device', '')  # 音频设备
        
        # 获取参数
        self.format = self.get_parameter('format').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.channels = self.get_parameter('channels').value
        self.device = self.get_parameter('device').value
        
        # 创建临时目录
        self.temp_dir = tempfile.mkdtemp(prefix='audio_play_')
        self.get_logger().info(f'创建临时目录: {self.temp_dir}')
        
        # 创建订阅者，订阅音频数据
        self.audio_subscription = self.create_subscription(
            AudioData,
            '/audio_generated',
            self.audio_callback,
            10
        )
        
        # 创建发布者，发布音频播放完成事件
        self.playback_complete_publisher = self.create_publisher(
            Empty,
            '/audio_playback_complete',
            10
        )
        
        # 当前播放进程
        self.current_process = None
        self.play_lock = threading.Lock()
        
        self.get_logger().info('音频播放节点已初始化')
        self.get_logger().info(f'音频格式: {self.format}, 采样率: {self.sample_rate}, 声道数: {self.channels}')
        if self.device:
            self.get_logger().info(f'使用音频设备: {self.device}')
        else:
            self.get_logger().info('使用默认音频设备')
    
    def audio_callback(self, msg):
        """
        处理接收到的音频数据，播放到扬声器
        """
        if not msg.data:
            self.get_logger().warn("收到空音频数据，忽略")
            return
        
        try:
            # 生成唯一的临时文件名
            timestamp = int(time.time() * 1000)
            temp_file = os.path.join(self.temp_dir, f"audio_{timestamp}.{self.format}")
            
            # 将音频数据写入临时文件
            with open(temp_file, 'wb') as f:
                f.write(msg.data)
            
            self.get_logger().info(f"已将{len(msg.data)}字节的音频数据保存到临时文件: {temp_file}")
            
            # 播放音频文件
            self.play_audio_file(temp_file)
            
        except Exception as e:
            self.get_logger().error(f"处理音频数据时发生错误: {str(e)}")
    
    def play_audio_file(self, file_path):
        """
        使用系统命令播放音频文件
        """
        with self.play_lock:
            # 如果有正在播放的进程，先停止它
            if self.current_process and self.current_process.poll() is None:
                self.get_logger().info("停止当前正在播放的音频")
                self.current_process.terminate()
                try:
                    self.current_process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    self.current_process.kill()
            
            # 根据文件格式选择播放命令
            if self.format.lower() == 'mp3':
                cmd = ['mpg123']
            elif self.format.lower() in ['wav', 'wave']:
                cmd = ['aplay']
            else:
                cmd = ['ffplay', '-nodisp', '-autoexit']
            
            # 添加设备参数（如果指定）
            if self.device and self.format.lower() in ['wav', 'wave']:
                cmd.extend(['-D', self.device])
            
            # 添加文件路径
            cmd.append(file_path)
            
            self.get_logger().info(f"播放音频: {' '.join(cmd)}")
            
            try:
                # 启动播放进程
                self.current_process = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
                
                # 创建一个线程来处理进程完成
                def process_complete():
                    stdout, stderr = self.current_process.communicate()
                    exit_code = self.current_process.returncode
                    
                    if exit_code != 0:
                        self.get_logger().error(f"音频播放失败，退出码: {exit_code}")
                        if stderr:
                            self.get_logger().error(f"错误输出: {stderr.decode('utf-8', errors='replace')}")
                    else:
                        self.get_logger().info(f"音频播放完成: {file_path}")
                        
                        # 发布音频播放完成事件
                        self.playback_complete_publisher.publish(Empty())
                    
                    # 播放完成后删除临时文件
                    try:
                        os.remove(file_path)
                        self.get_logger().debug(f"已删除临时文件: {file_path}")
                    except Exception as e:
                        self.get_logger().warning(f"删除临时文件失败: {str(e)}")
                
                thread = threading.Thread(target=process_complete)
                thread.daemon = True
                thread.start()
                
            except Exception as e:
                self.get_logger().error(f"启动音频播放进程时发生错误: {str(e)}")
    
    def cleanup_temp_files(self):
        """
        清理临时文件
        """
        try:
            for file in Path(self.temp_dir).glob("audio_*"):
                try:
                    file.unlink()
                except Exception as e:
                    self.get_logger().warning(f"删除临时文件失败: {str(e)}")
            
            os.rmdir(self.temp_dir)
            self.get_logger().info(f"已清理临时目录: {self.temp_dir}")
        except Exception as e:
            self.get_logger().error(f"清理临时文件时发生错误: {str(e)}")
    
    def destroy(self):
        """
        清理资源
        """
        # 停止当前播放
        if self.current_process and self.current_process.poll() is None:
            self.get_logger().info("停止当前正在播放的音频")
            self.current_process.terminate()
            try:
                self.current_process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self.current_process.kill()
        
        # 清理临时文件
        self.cleanup_temp_files()
        
        self.get_logger().info("音频播放节点已关闭")


def main(args=None):
    rclpy.init(args=args)
    
    node = AudioPlayNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
