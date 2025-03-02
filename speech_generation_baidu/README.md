# Speech Generation Baidu

这是一个ROS2功能包，使用百度语音合成API将文本转换为语音。它订阅`/llm_response`话题上的文本消息，并将合成的音频发布到`/audio_generated`话题上。

## 功能

- 订阅`/llm_response`话题，接收文本消息
- 使用百度语音合成API将文本转换为语音
- 将合成的音频数据发布到`/audio_generated`话题
- 支持调整发音人、语速、音调、音量等参数
- 支持多种音频格式(mp3, pcm, wav)

## 依赖

- ROS2
- Python 3
- rclpy
- std_msgs
- audio_common_msgs

## 安装

1. 克隆此功能包到您的工作空间的src目录
2. 编译工作空间
```bash
cd ~/your_workspace
colcon build --packages-select speech_generation_baidu
```

## 使用方法

### 1. 获取百度语音合成API的密钥

您需要在[百度AI开放平台](https://ai.baidu.com/)注册并创建一个应用，获取API Key和Secret Key。

### 2. 启动节点

使用launch文件启动节点，并传入API密钥：

```bash
ros2 launch speech_generation_baidu speech_generation_baidu.launch.py baidu_api_key:=your_api_key baidu_secret_key:=your_secret_key
```

您也可以调整其他参数：

```bash
ros2 launch speech_generation_baidu speech_generation_baidu.launch.py baidu_api_key:=your_api_key baidu_secret_key:=your_secret_key per:=1 spd:=7 pit:=6 vol:=8 aue:=6
```

### 3. 参数说明

- `baidu_api_key`: 百度语音API的API Key
- `baidu_secret_key`: 百度语音API的Secret Key
- `per`: 发音人选择
  - 0: 度小美（女声，中英文）
  - 1: 度小宇（男声，中英文）
  - 3: 度逍遥（男声，中英文）
  - 4: 度丫丫（女童声，中英文）
  - 5: 度小娇（女声，中英文）
  - 103: 度米朵（女声，中英文）
  - 106: 度博文（男声，中英文）
  - 110: 度小童（童声，中英文）
  - 111: 度小萌（女声，中英文）
- `spd`: 语速，取值0-15，默认为5中语速
- `pit`: 音调，取值0-15，默认为5中音调
- `vol`: 音量，取值0-9，默认为5中音量
- `aue`: 文件格式
  - 3: mp3 (默认)
  - 4: pcm-16k
  - 5: pcm-8k
  - 6: wav

### 4. 测试

您可以通过发布消息到`/llm_response`话题来测试功能：

```bash
ros2 topic pub /llm_response std_msgs/msg/String "data: '这是一条测试消息，用于测试百度语音合成功能。'"
```

然后，您可以监听`/audio_generated`话题来获取合成的音频数据：

```bash
ros2 topic echo /audio_generated
```

### 5. 使用音频播放节点

要听到合成的语音，您可以使用`audio_play`节点（来自audio_common包）播放音频数据：

```bash
ros2 run audio_play audio_play_node __params:=audio_play_config.yaml
```

其中，`audio_play_config.yaml`中配置了订阅的话题为`/audio_generated`。

## 故障排除

如果遇到问题：

1. 确保百度API密钥正确设置
2. 检查网络连接，确保可以访问百度API
3. 查看节点日志获取详细错误信息
4. 调试文件保存在`~/speech_generation_baidu_debug/`目录下

## 许可证

TODO

## 作者

TODO
