# 机器人语音系统Pipeline详解

## 整体架构

整个语音系统由以下几个主要组件按顺序组成：

1. **音频捕获 (audio_capture_filter)** - 带高通滤波器的麦克风捕获模块
2. **语音识别 (speech_recognition_baidu)** - 基于百度API的语音识别模块
3. **大语言模型 (llm_bytedance)** - 基于火山引擎的大语言模型处理模块
4. **语音合成 (speech_generation_baidu)** - 基于百度API的语音合成模块
5. **音频播放 (audio_play_python)** - 音频播放模块
6. **音频录制 (audio_recorder.py)** - 用于调试的音频录制模块

## 数据流向

整个系统的数据流向如下：

1. **麦克风捕获音频** → 发布到 `/audio/audio` 话题
2. **语音识别** → 订阅 `/audio/audio` 话题，识别语音，发布到语音识别结果话题
3. **LLM处理** → 接收识别的文本，进行处理，生成回复文本
4. **语音合成** → 接收LLM生成的文本，合成语音，发布到 `/audio_generated` 话题
5. **音频播放** → 订阅 `/audio_generated` 话题，播放音频
6. **音频录制** → 订阅 `/audio/audio` 话题，保存原始音频数据用于调试

## 各模块详细说明

### 1. 音频捕获模块 (audio_capture_filter)

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
  - `language`: 识别语言 (默认: `en_us`)
  - `buffer_size`: 音频缓冲区大小 (默认: `32000`)
  - `silence_threshold`: 静音检测阈值 (默认: `200`)
  - `silence_duration`: 判断语音结束的静音持续时间 (默认: `1.5`秒)
  - `baidu_api_key` 和 `baidu_secret_key`: 百度API密钥
- **输入话题**: `/audio/audio`
- **输出**: 识别的文本结果

### 3. 大语言模型模块 (llm_bytedance)

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

### 6. 音频录制模块 (audio_recorder.py)

- **功能**：记录原始音频数据用于调试
- **主要参数**：
  - `channels`: 声道数 (默认: `2`)
  - `sample_width`: 采样宽度 (默认: `2`字节)
  - `sample_rate`: 采样率 (默认: `16000` Hz)
  - `max_duration`: 最大录音时长 (默认: `60`秒)
- **输入话题**: `/audio/audio`
- **输出**: 保存的WAV文件和原始二进制数据

## 启动方式

整个系统通过 `voice_system.launch.py` 按顺序启动所有组件：

```bash
ros2 launch robot_voice_launcher voice_system.launch.py
```

可选参数：
- `enable_filter`: 是否启用高通滤波器 (默认: `true`)
- `cutoff_frequency`: 高通滤波器截止频率 (默认: `100.0` Hz)

例如，禁用滤波器启动：
```bash
ros2 launch robot_voice_launcher voice_system.launch.py enable_filter:=false
```

## 系统依赖

- **音频播放依赖**:
  - mpg123: 用于播放MP3格式音频
  - aplay: 用于播放WAV格式音频
  - ffplay: 作为备选播放器，支持多种格式

- **API依赖**:
  - 百度语音API (语音识别和合成)
  - 火山引擎API (大语言模型)
