# Speech Recognition StepFun

这是一个ROS2功能包，用于使用阶跃星辰（StepFun）的语音识别API进行语音识别。该节点订阅`/audio/audio`话题的音频数据，并发布识别出的文本到`/speech_to_text`话题。

## 功能

- 订阅ROS2音频消息
- 使用语音活动检测（VAD）检测完整的语音片段
- 使用阶跃星辰API进行语音识别
- 发布识别结果到ROS2话题

## 依赖项

- ROS2
- Python 3
- numpy
- requests

## 安装

1. 将此功能包克隆到您的ROS2工作空间的`src`目录中
2. 构建工作空间：
   ```bash
   cd ~/your_workspace
   colcon build --packages-select speech_recognition_stepfun
   ```
3. 刷新环境：
   ```bash
   source install/setup.bash
   ```

## 使用方法

### 测试阶跃星辰API

在使用ROS2节点之前，您可以使用提供的测试脚本测试阶跃星辰API：

```bash
python3 test_stepfun_api.py --api_key YOUR_API_KEY --audio_file path/to/your/audio.wav
```

### 运行节点

使用提供的启动文件运行节点：

```bash
ros2 launch speech_recognition_stepfun speech_recognition_stepfun.launch.py stepfun_api_key:=YOUR_API_KEY
```

### 参数

以下是可以在启动文件中配置的参数：

- `language`: 语音识别的语言（默认：zh-cn）
- `buffer_size`: 音频缓冲区大小（默认：32000）
- `silence_threshold`: 静音检测阈值（默认：200）
- `min_utterance_length`: 最小语句长度（默认：16000）
- `silence_duration`: 静音持续时间，用于判定句子结束（默认：1.5秒）
- `stepfun_api_key`: 阶跃星辰API密钥（必需）
- `use_proxy`: 是否使用代理（默认：false）
- `http_proxy`: HTTP代理地址
- `https_proxy`: HTTPS代理地址

## 话题

### 订阅

- `/audio/audio` (audio_common_msgs/AudioData): 原始音频数据

### 发布

- `/speech_to_text` (std_msgs/String): 识别出的文本

## 注意事项

- 您需要获取阶跃星辰API的密钥才能使用此功能包
- 默认情况下，节点会在用户主目录下创建一个`speech_recognition_stepfun_debug`目录，用于存储调试信息
