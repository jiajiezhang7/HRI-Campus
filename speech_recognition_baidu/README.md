# Speech Recognition Baidu

这是一个ROS2功能包，使用百度语音识别API进行语音识别。该功能包订阅ROS2话题`/audio/audio`接收音频数据，并将识别结果发布到`/recognized_text`和`/speech_to_text`话题。

## 功能特点

- 使用百度语音识别REST API
- 支持中文和英文语音识别
- 语音活动检测 (VAD)，自动检测语音开始和结束
- 支持代理设置，方便在网络受限环境中使用
- 保存原始音频数据和识别后的语音片段，方便调试

## 依赖项

- ROS2
- audio_common_msgs
- numpy
- wave

## 使用方法

### 1. 获取百度语音识别API密钥

在使用此功能包之前，您需要先在百度AI开放平台申请语音识别服务，获取API Key和Secret Key：

1. 访问[百度AI开放平台](https://ai.baidu.com/)
2. 注册并登录账号
3. 创建应用，选择语音技术 > 语音识别
4. 获取应用的API Key和Secret Key

### 2. 启动语音识别节点

使用launch文件启动语音识别节点：

```bash
ros2 launch speech_recognition_baidu speech_recognition_baidu.launch.py baidu_api_key:=YOUR_API_KEY baidu_secret_key:=YOUR_SECRET_KEY
```

或者直接运行节点：

```bash
ros2 run speech_recognition_baidu speech_recognition_baidu_node --ros-args -p baidu_api_key:=YOUR_API_KEY -p baidu_secret_key:=YOUR_SECRET_KEY
```

### 3. 参数说明

- `language`: 语音识别的语言，支持 'zh-cn' (中文) 和 'en-us' (英文)，默认为 'zh-cn'
- `buffer_size`: 音频缓冲区大小，默认为 32000
- `silence_threshold`: 静音检测阈值，默认为 200
- `min_utterance_length`: 最小语句长度（字节），默认为 16000
- `silence_duration`: 静音持续时间（秒）判定为句子结束，默认为 1.5
- `baidu_api_key`: 百度语音识别API Key
- `baidu_secret_key`: 百度语音识别Secret Key
- `use_proxy`: 是否使用代理服务器连接API，默认为 false
- `http_proxy`: HTTP代理服务器地址
- `https_proxy`: HTTPS代理服务器地址

## 话题

### 订阅的话题

- `/audio/audio` (audio_common_msgs/AudioData): 原始音频数据

### 发布的话题

- `/recognized_text` (std_msgs/String): 识别出的文本
- `/speech_to_text` (std_msgs/String): 识别出的文本（兼容性话题）

## 调试信息

语音识别节点会在用户主目录下创建 `~/speech_recognition_baidu_debug` 目录，用于保存以下调试信息：

- 原始音频数据: `raw_audio_YYYYMMDD-HHMMSS.bin`
- 识别的语音片段: `utterance_YYYYMMDD-HHMMSS.wav`

## 与科大讯飞API的区别

与科大讯飞API相比，百度语音识别API使用的是HTTP REST API方式，而不是WebSocket方式。主要区别包括：

1. 连接方式：百度使用HTTP POST请求，科大讯飞使用WebSocket连接
2. 参数格式：两者的参数格式和域名不同
3. 签名算法：百度的签名生成方式相对简单
4. 数据传输：百度一次性发送整个音频文件，科大讯飞通过WebSocket分段传输

## 许可证

待定
