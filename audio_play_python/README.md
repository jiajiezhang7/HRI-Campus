# Audio Play Python

这是一个ROS2节点，用于将音频数据播放到系统扬声器。它订阅`/audio_generated`话题，接收`audio_common_msgs/AudioData`类型的消息，并使用系统命令将音频播放出来。

## 功能

- 订阅`/audio_generated`话题，接收音频数据
- 支持MP3和WAV格式的音频播放
- 使用系统命令（mpg123、aplay、ffplay等）作为音频处理后端
- 可配置音频参数（格式、设备、声道数、采样率等）

## 系统依赖

本功能包依赖以下系统工具（根据音频格式需要安装相应的播放器）：

- `mpg123`：用于播放MP3格式的音频
- `aplay`：用于播放WAV格式的音频
- `ffplay`：作为备选播放器，支持多种格式

可以通过以下命令安装这些依赖：

```bash
sudo apt-get update
sudo apt-get install -y mpg123 alsa-utils ffmpeg
```

## ROS2依赖

- `rclpy`：ROS2的Python客户端库
- `audio_common_msgs`：音频相关的消息定义
- `speech_generation_baidu`：百度语音合成节点（提供`/audio_generated`话题）

## 使用方法

### 单独启动音频播放节点

```bash
ros2 launch audio_play_python audio_play.launch.py
```

### 参数配置

可以在启动时配置以下参数：

- `format`：音频格式，支持`mp3`和`wav`，默认为`mp3`
- `device`：音频设备，留空使用默认设备
- `channels`：声道数，默认为`1`
- `sample_rate`：采样率，默认为`16000`

例如：

```bash
ros2 launch audio_play_python audio_play.launch.py format:=wav sample_rate:=44100
```

## 与语音生成节点一起使用

要将此节点与`speech_generation_baidu`节点一起使用，可以分别启动两个节点：

```bash
# 启动语音生成节点
ros2 launch speech_generation_baidu speech_generation_baidu.launch.py

# 启动音频播放节点
ros2 launch audio_play_python audio_play.launch.py
```

这样，当语音生成节点在`/audio_generated`话题上发布音频数据时，音频播放节点会自动接收并播放这些数据。

## 工作原理

1. 节点订阅`/audio_generated`话题，接收音频数据
2. 将接收到的音频数据保存为临时文件
3. 根据音频格式选择合适的命令行工具播放音频
4. 播放完成后自动删除临时文件

## 故障排除

如果遇到音频播放问题，请检查：

1. 系统音量是否已打开
2. 是否已安装所需的命令行工具（mpg123、aplay或ffplay）
3. 音频格式是否与配置的格式匹配
4. 是否有权限访问音频设备（可能需要将用户添加到`audio`组）

如果需要调试，可以查看节点的日志输出：

```bash
ros2 run audio_play_python audio_play_node --ros-args --log-level debug
