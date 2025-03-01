# 语音识别 ROS2 包

这个包提供了使用科大讯飞多语种语音识别API进行语音识别的ROS2节点。它可以订阅来自音频捕获节点的音频数据，并将识别出的文本发布到ROS2话题。
（11201错误 - 明天再试试）

## 功能

- 订阅 `/audio/audio` 话题，接收音频数据
- 使用科大讯飞WebSocket API进行语音识别
- 将识别出的文本发布到 `/recognized_text` 和 `/speech_to_text` 话题
- 支持中文和英文语音识别

## 依赖

- ROS2 Iron
- audio_common 包
- websocket-client
- numpy

## 安装

1. 确保已安装必要的Python依赖：

```bash
pip install websocket-client numpy
```

2. 克隆并构建包：

```bash
cd ~/microp_ws/src
# 已经创建了speech_recognition包
cd ..
colcon build --packages-select speech_recognition
source install/setup.bash
```

## 使用方法

### 配置科大讯飞API

使用前需要先在科大讯飞开放平台(https://www.xfyun.cn/)注册账号并创建应用，获取以下信息：
- AppID
- API Key
- API Secret

### 启动语音识别系统

使用launch文件启动整个系统（包括音频捕获和语音识别）：

```bash
ros2 launch speech_recognition speech_recognition.launch.py xf_app_id:=YOUR_APP_ID xf_api_key:=YOUR_API_KEY xf_api_secret:=YOUR_API_SECRET
```

### 参数配置

launch文件支持以下参数：

- `language`: 语音识别的语言，可以是 `zh-cn`（中文，默认）或 `en-us`（英文）
- `buffer_size`: 音频缓冲区大小，默认为32000（约1秒的音频数据）
- `xf_app_id`: 科大讯飞应用ID
- `xf_api_key`: 科大讯飞API Key
- `xf_api_secret`: 科大讯飞API Secret
- `use_proxy`: 是否使用代理，默认为false
- `http_proxy`: HTTP代理地址
- `https_proxy`: HTTPS代理地址

例如，使用英文识别：

```bash
ros2 launch speech_recognition speech_recognition.launch.py language:=en-us xf_app_id:=YOUR_APP_ID xf_api_key:=YOUR_API_KEY xf_api_secret:=YOUR_API_SECRET
```

### 查看识别结果

在另一个终端中运行以下命令查看识别出的文本：

```bash
ros2 topic echo /recognized_text
# 或
ros2 topic echo /speech_to_text
```

## 技术说明

本节点使用科大讯飞WebSocket API进行语音识别，相比HTTP REST API方式有以下优势：
1. 支持实时流式识别
2. 可以处理更长的音频
3. 响应速度更快
4. 符合官方推荐的接入方式

## 故障排除

1. 如果识别准确率低，可以尝试调整麦克风设置或环境噪音。

2. 如果没有识别结果，检查音频捕获节点是否正常工作：

```bash
ros2 topic echo /audio/audio
```

3. 如果遇到网络连接问题，可以尝试配置代理：

```bash
ros2 launch speech_recognition speech_recognition.launch.py use_proxy:=true http_proxy:=http://your-proxy:port https_proxy:=https://your-proxy:port
```

4. 查看节点日志获取更多调试信息：

```bash
ros2 log level set /speech_recognition_node debug
```
