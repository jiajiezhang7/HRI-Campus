# Speech Generation StepFun

这是一个ROS2功能包，用于使用阶跃星辰的语音合成API将文本转换为语音。该节点订阅`/llm_response`话题的文本消息，并将合成的音频发布到`/audio_generated`话题。

## 功能

- 订阅`/llm_response`话题，接收要转换为语音的文本
- 调用阶跃星辰语音合成API生成音频
- 将生成的音频发布到`/audio_generated`话题
- 支持配置多种参数，如音色、语速、音量等
- 支持HTTP/HTTPS代理配置

## 安装

### 前提条件

- ROS2环境（推荐Humble或更新版本）
- Python 3.8+
- 阶跃星辰API密钥

### 依赖项

- rclpy
- std_msgs
- audio_common_msgs
- requests

### 构建

```bash
cd ~/your_workspace
colcon build --packages-select speech_generation_stepfun
source install/setup.bash
```

## 使用方法

### 启动节点

使用launch文件启动节点：

```bash
ros2 launch speech_generation_stepfun speech_generation_stepfun.launch.py stepfun_api_key:=your_api_key
```

或者直接运行节点：

```bash
ros2 run speech_generation_stepfun speech_generation_stepfun_node --ros-args -p stepfun_api_key:=your_api_key
```

### 参数配置

以下是可配置的参数：

| 参数名 | 类型 | 默认值 | 描述 |
|--------|------|--------|------|
| stepfun_api_key | string | '' | 阶跃星辰API密钥 |
| voice | string | 'cixingnansheng' | 音色选择 |
| response_format | string | 'mp3' | 音频格式（支持wav,mp3,flac,opus） |
| speed | float | 1.0 | 语速（范围0.5~2） |
| volume | float | 1.0 | 音量（范围0.1~2.0） |
| use_proxy | bool | false | 是否使用代理 |
| http_proxy | string | '' | HTTP代理地址 |
| https_proxy | string | '' | HTTPS代理地址 |

### 话题

#### 订阅的话题

- `/llm_response` (std_msgs/String): 要转换为语音的文本

#### 发布的话题

- `/audio_generated` (audio_common_msgs/AudioData): 生成的音频数据

## 调试

生成的音频文件和错误日志会保存在`~/speech_generation_stepfun_debug`目录下，以便于调试。

## 许可证

TODO: 添加许可证信息
