# 机器人语音系统启动器

这个ROS2软件包用于按顺序启动所有语音交互相关的节点，实现完整的语音交互功能。

## 功能特点

按照以下顺序启动节点：

1. `audio_capture` - 麦克风音频捕获 (comica_mic_capture.launch.py)
2. `speech_recognition_baidu` - 百度语音识别
3. `llm_bytedance` - 字节跳动大语言模型
4. `speech_generation_baidu` - 百度语音合成
5. `audio_recorder.py` - 音频记录脚本

## 使用方法

### 依赖安装

确保已安装以下ROS2软件包：
- audio_capture
- speech_recognition_baidu
- llm_bytedance
- speech_generation_baidu
- sudo apt-get install libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-bad (for audio_capture_filter_node.cpp)
### 构建

```bash
cd ~/microp_ws
colcon build --packages-select robot_voice_launcher
source install/setup.bash
```

### 运行

```bash
ros2 launch robot_voice_launcher voice_system.launch.py
```

### 配置参数

如需修改各个组件的默认参数，可以在`voice_system.launch.py`文件中的相应启动配置部分进行调整。

## 工作原理

该启动器使用ROS2 launch系统的TimerAction功能，为每个组件设定启动时间，确保它们按照预定的顺序依次启动。这样能够保证语音交互系统的各个组件正常工作，并相互之间能够正确通信。

- 每个组件之间有2秒的延迟，以确保前一个组件有足够的时间完成初始化
- 所有组件都会在同一个终端窗口中输出日志信息，便于监控系统运行状态
