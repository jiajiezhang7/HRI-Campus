# 人机交互系统详解

## 系统概述

这个工作区实现了一个基于ROS2的人机交互系统，主要包括视觉感知、语音交互和动画显示三大部分。系统能够检测人脸，与人进行语音对话，并在第二屏幕上显示动画。整个系统设计用于电梯场景，可以根据电梯楼层信息与人进行交互。

## 整体架构

系统由以下几个主要子系统组成：

1. **视觉感知系统** - 摄像头、鱼眼处理、人体检测和舵机控制
2. **语音交互系统** - 音频捕获、语音识别、大语言模型、语音合成和音频播放
3. **动画显示系统** - 在第二屏幕上显示机器人状态动画
4. **电梯交互系统** - 提供电梯楼层信息
5. **系统集成与协调** - 协调各个子系统的工作

## 数据流向

整个系统的数据流向如下：

### 视觉感知系统

1. **摄像头采集图像** → 发布到 `/insta360_air/image_raw` 话题
2. **鱼眼处理** → 订阅原始图像，将双鱼眼图像拼接成全景图，发布到 `/insta360_air/image_get` 话题
3. **人体检测** → 订阅全景图，检测人体和人脸，计算人脸角度，发布到 `/face_angle` 话题
4. **舵机控制** → 订阅人脸角度，控制舵机旋转，使摄像头朝向人脸

### 语音交互系统

1. **麦克风捕获音频** → 发布到 `/audio/audio` 话题
2. **语音识别** → 订阅音频数据，识别语音，发布到 `/speech_to_text` 话题
3. **LLM处理** → 订阅语音识别结果，进行处理，发布到 `/llm_response` 话题
4. **语音合成** → 订阅LLM响应，合成语音，发布到 `/audio_generated` 话题
5. **音频播放** → 订阅合成的音频，播放音频，完成后发布到 `/audio_playback_complete` 话题

### 动画显示系统

1. **TTS状态发布节点** → 订阅 `/audio_generated` 和 `/audio_playback_complete` 话题，发布到 `/tts_status` 话题
2. **网页动画显示** → 通过rosbridge订阅 `/tts_status` 话题，切换动画状态

### 系统协调

1. **交互协调器节点** → 订阅 `/face_angle`、`/llm_response` 和 `/audio_playback_complete` 话题，发布到 `/continue_detection` 话题
2. **主动发问节点** → 订阅 `/dummy_level` 话题，提供 `/active_questioning/trigger_question` 服务
3. **麦克风静音控制节点** → 在机器人说话时静音麦克风

## 各模块详细说明

### 视觉感知系统

#### 1. 摄像头模块 (usb_cam)

- **功能**：提供摄像头驱动，获取原始图像数据
- **输出话题**: `/insta360_air/image_raw`

#### 2. 鱼眼处理模块 (fisheye_process)

- **功能**：将双鱼眼图像拼接成全景图
- **工作原理**：使用OpenCV进行图像处理和拼接
- **输入话题**: `/insta360_air/image_raw`
- **输出话题**: `/insta360_air/image_get`

#### 3. 人体检测模块 (human_detect)

- **功能**：检测人体和人脸，计算人脸角度
- **主要组件**：
  - YOLOv8: 用于人体检测
  - MTCNN: 用于人脸检测
- **输入话题**: `/insta360_air/image_get`
- **输出话题**: `/face_angle`
- **订阅话题**: `/continue_detection` (控制是否继续检测)

#### 4. 舵机控制模块 (servo)

- **功能**：控制舵机旋转，使摄像头朝向人脸
- **工作原理**：通过串口将人脸角度发送给Arduino控制的舵机
- **输入话题**: `/face_angle`

### 语音交互系统

#### 1. 音频捕获模块 (audio_capture_filter)

- **功能**：从麦克风捕获音频，应用高通滤波器降噪，发布音频数据
- **主要参数**：
  - `device`: 音频设备标识符 (默认: `plughw:3,0`)
  - `format`: 音频格式 (默认: `wave`)
  - `channels`: 声道数 (默认: `2`)
  - `sample_rate`: 采样率 (默认: `16000` Hz)
  - `enable_filter`: 是否启用高通滤波器 (默认: `true`)
  - `cutoff_frequency`: 高通滤波器截止频率 (默认: `100.0` Hz)
- **输出话题**: `/audio/audio`
- **滤波器原理**：
  - 高通滤波器过滤掉低于截止频率的信号，保留高于截止频率的信号
  - 用于去除低频噪声（如风噪、呼吸声、空调声等）
  - 实现的是一阶高通滤波器，计算公式：`y[n] = alpha * (y[n-1] + x[n] - x[n-1])`

### 2. 语音识别模块 (speech_recognition_baidu)

- **功能**：接收音频数据，使用百度API进行语音识别
- **主要参数**：
  - `language`: 识别语言 (默认: `en-us`)
  - `buffer_size`: 音频缓冲区大小 (默认: `32000`)
  - `silence_threshold`: 静音检测阈值 (默认: `200`)
  - `silence_duration`: 判断语音结束的静音持续时间 (默认: `1.5`秒)
  - `baidu_api_key` 和 `baidu_secret_key`: 百度API密钥
- **输入话题**: `/audio/audio`
- **输出**: 识别的文本结果

#### 3. 大语言模型模块 (llm_bytedance)

- **功能**：处理语音识别的文本，生成回复
- **主要参数**：
  - `model_id`: 火山引擎大模型ID (默认: `ep-20250213183336-ngs4d`)
  - 环境变量 `ARK_API_KEY`: API密钥
- **输入**: 语音识别的文本
- **输出**: 生成的回复文本

### 4. 语音合成模块 (speech_generation_baidu)

- **功能**：将文本转换为语音
- **主要参数**：
  - `baidu_api_key` 和 `baidu_secret_key`: 百度API密钥
  - `per`: 发音人选择 (默认: `4226`)
  - `spd`: 语速 (默认: `5`)
  - `pit`: 音调 (默认: `5`)
  - `vol`: 音量 (默认: `5`)
  - `aue`: 文件格式 (默认: `3` - mp3)
- **输入**: LLM生成的文本
- **输出话题**: `/audio_generated`

### 5. 音频播放模块 (audio_play_python)

- **功能**：播放合成的语音
- **主要参数**：
  - `format`: 音频格式 (默认: `mp3`)
  - `device`: 音频设备 (默认: 系统默认设备)
  - `channels`: 声道数 (默认: `1`)
  - `sample_rate`: 采样率 (默认: `16000` Hz)
- **输入话题**: `/audio_generated`
- **工作流程**:
  1. 接收音频数据
  2. 保存为临时文件
  3. 根据格式选择播放工具 (mpg123, aplay, ffplay)
  4. 播放完成后删除临时文件

#### 6. 音频录制模块 (audio_recorder.py)

- **功能**：记录原始音频数据用于调试
- **主要参数**：
  - `channels`: 声道数 (默认: `2`)
  - `sample_width`: 采样宽度 (默认: `2`字节)
  - `sample_rate`: 采样率 (默认: `16000` Hz)
  - `max_duration`: 最大录音时长 (默认: `60`秒)
- **输入话题**: `/audio/audio`
- **输出**: 保存的WAV文件和原始二进制数据

### 动画显示系统

#### 1. TTS状态发布节点 (tts_status_publisher)

- **功能**：监控TTS状态并发布信息到`/tts_status`话题
- **工作原理**：监听音频生成和播放完成事件，更新机器人说话状态
- **输入话题**: `/audio_generated` 和 `/audio_playback_complete`
- **输出话题**: `/tts_status`

#### 2. 网页动画显示 (character_display.html)

- **功能**：在第二屏幕上显示机器人状态动画
- **工作原理**：使用rosbridge订阅`/tts_status`话题，根据机器人是否正在说话切换静态图片和动画
- **输入话题**: `/tts_status`

### 电梯交互系统

#### 1. 电梯楼层信息发布节点 (dummy_level_publisher)

- **功能**：模拟电梯楼层信息发布
- **工作原理**：随机生成电梯方向和楼层信息，并定期发布
- **输出话题**: `/dummy_level`

### 系统集成与协调

#### 1. 交互协调器节点 (interaction_coordinator_node)

- **功能**：协调摄像头系统与语音交互系统
- **工作原理**：
  - 监听人脸角度，触发主动发问
  - 监听大语言模型响应，判断交互是否结束
  - 控制人脸检测的开启和关闭
- **输入话题**: `/face_angle`、`/llm_response` 和 `/audio_playback_complete`
- **输出话题**: `/continue_detection`
- **服务客户端**: `/active_questioning/trigger_question`

#### 2. 主动发问节点 (active_questioning_node)

- **功能**：根据电梯楼层信息生成问题文本
- **工作原理**：
  - 订阅电梯楼层信息
  - 提供服务接口供其他节点调用
  - 根据电梯方向和楼层生成不同的问题文本
- **输入话题**: `/dummy_level`
- **输出话题**: `/llm_response`
- **服务**: `/active_questioning/trigger_question`

#### 3. 麦克风静音控制节点 (mic_mute_node)

- **功能**：在机器人说话时静音麦克风，避免自我干扰
- **工作原理**：
  - 监听音频生成和播放完成事件
  - 监听交互状态
  - 控制麦克风静音状态
- **输入话题**: `/audio_generated`、`/audio_playback_complete`、`/continue_detection` 和 `/llm_response`

## 启动方式

### 完整系统启动

整个系统可以通过集成启动文件启动：

```bash
ros2 launch robot_voice_launcher integrated_interaction.launch.py
```

这个启动文件会按以下顺序启动各个组件：

1. 首先启动动画显示系统
2. 等待1秒后启动摄像头系统和电梯信息发布节点
3. 等待3秒后启动语音系统
4. 等待7秒后启动交互协调器节点

### 单独启动语音系统

如果只需要启动语音交互系统，可以使用：

```bash
ros2 launch robot_voice_launcher voice_system_stepfun.launch.py
```

可选参数：
- `enable_filter`: 是否启用高通滤波器 (默认: `true`)
- `cutoff_frequency`: 高通滤波器截止频率 (默认: `1500.0` Hz)
- `silence_threshold`: 语音识别静音阈值 (默认: `50`)

### 单独启动摄像头系统

如果只需要启动摄像头系统，可以使用：

```bash
ros2 launch robot_voice_launcher camera_system.launch.py
```

### 单独启动动画显示系统

如果只需要启动动画显示系统，可以使用：

```bash
ros2 launch robot_animation_display animation_display.launch.py
```

### 调试启动

用于调试语音交互的启动文件：

```bash
ros2 launch robot_voice_launcher debug_voice_interaction.launch.py
```

这个启动文件会启动语音系统、电梯信息发布节点和服务触发节点，用于测试主动发问功能。

## 系统依赖

### ROS2基础依赖 (Iron)

- **ROS2核心依赖**:
  - `rclpy` - Python客户端库
  - `rclcpp` - C++客户端库（用于audio_capture）
  - `rclcpp_components` - ROS2组件
  - `std_msgs` - 标准消息类型
  - `std_srvs` - 标准服务类型
  - `launch` - 启动系统
  - `launch_ros` - ROS2启动系统
  - `launch_xml` - XML格式启动文件支持
  - `ament_cmake` - CMake构建工具
  - `ament_python` - Python构建工具
  - `rosidl_default_generators` - 消息生成器
  - `rosidl_default_runtime` - 消息运行时

### 音频相关依赖

- **audio_common相关依赖**:
  - `audio_common_msgs` - 音频消息类型
  - `audio_capture` - 音频捕获节点
  - `audio_play` - 音频播放节点（C++版本）
  - `sound_play` - 声音播放工具

- **GStreamer相关依赖**:
  - `libgstreamer1.0-dev`
  - `libgstreamer-plugins-base1.0-dev`
  - `libgstreamer-plugins-bad1.0-dev`
  - `gstreamer1.0`
  - `gstreamer1.0-alsa`
  - `gstreamer1.0-plugins-base`
  - `gstreamer1.0-plugins-good`
  - `gstreamer1.0-plugins-ugly`
  - `gstreamer1.0-plugins-bad`

- **其他系统依赖**:
  - `boost` - C++库
  - `diagnostic_updater` - 诊断工具

### Python依赖

- **Python库依赖**:
  - `numpy` - 用于音频处理
  - `requests` - 用于API请求（llm_bytedance）
  - `wave` - 用于WAV文件处理
  - `urllib` - 用于HTTP请求
  - `json` - 用于JSON处理
  - `base64` - 用于编码解码

### 视觉感知系统依赖

- **图像处理依赖**:
  - OpenCV: 用于图像处理和拼接
  - cv_bridge: ROS和OpenCV之间的桥接
  - image_transport: 图像传输库

- **深度学习依赖**:
  - YOLOv8: 用于人体检测
  - MTCNN: 用于人脸检测
  - PyTorch: 深度学习框架

- **硬件依赖**:
  - pyserial: 用于串口通信
  - Arduino: 用于舵机控制

### 动画显示系统依赖

- **Web相关依赖**:
  - rosbridge_server: 提供ROS和Web之间的通信
  - roslib.js: JavaScript客户端库
  - HTML/CSS/JavaScript: 网页前端技术

### 系统工具依赖

- **音频播放依赖**:
  - mpg123: 用于播放MP3格式音频
  - aplay: 用于播放WAV格式音频
  - ffplay: 作为备选播放器，支持多种格式

- **API依赖**:
  - 百度语音API: 用于语音识别和合成
  - 火山引擎API: 用于大语言模型
  - 阶跳星辰API: 用于语音识别和合成

### 安装命令

您可以使用以下命令安装大部分依赖项：

```bash
# ROS2基础依赖
sudo apt-get install ros-iron-rclpy ros-iron-rclcpp ros-iron-rclcpp-components ros-iron-std-msgs ros-iron-std-srvs ros-iron-launch ros-iron-launch-ros ros-iron-launch-xml ros-iron-ament-cmake ros-iron-rosidl-default-generators ros-iron-rosidl-default-runtime

# 视觉感知系统依赖
sudo apt-get install ros-iron-cv-bridge ros-iron-image-transport ros-iron-usb-cam
sudo apt-get install python3-opencv
pip install ultralytics facenet-pytorch torch torchvision
pip install pyserial

# 动画显示系统依赖
sudo apt-get install ros-iron-rosbridge-server

# GStreamer相关依赖
sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-alsa gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly gstreamer1.0-plugins-bad

# 其他系统依赖
sudo apt-get install libboost-dev ros-iron-diagnostic-updater

# 系统工具
sudo apt-get install mpg123 alsa-utils ffmpeg

# Python依赖
pip install numpy requests wave urllib3
```

对于API密钥，您需要在环境中设置：
```bash
# 设置火山引擎API密钥
export ARK_API_KEY="您的火山引擎API密钥"

# 设置阶跳星辰API密钥
export STEPFUN_API_KEY="您的阶跳星辰API密钥"

# 百度API密钥需要在代码中配置或通过参数传入
```

## 系统工作流程

1. **启动流程**:
   - 首先启动动画显示系统
   - 然后启动摄像头系统和电梯信息发布节点
   - 接着启动语音系统
   - 最后启动交互协调器节点

2. **交互流程**:
   - 摄像头系统检测人脸，计算人脸角度
   - 交互协调器接收到人脸角度后触发主动发问
   - 主动发问节点根据电梯信息生成问题文本
   - 语音合成系统将文本转换为语音并播放
   - 麦克风捕获用户回答，语音识别系统将其转换为文本
   - 大语言模型处理文本并生成响应
   - 语音合成系统将响应转换为语音并播放
   - 动画显示系统根据语音状态显示相应动画

3. **控制流程**:
   - 交互协调器根据交互状态控制人脸检测的开启和关闭
   - 麦克风静音控制节点在机器人说话时静音麦克风
   - 舵机控制节点根据人脸角度控制摄像头朝向
