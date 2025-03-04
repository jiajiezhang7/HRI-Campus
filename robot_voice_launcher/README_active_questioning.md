# 机器人主动发问功能

## 功能说明

该功能允许机器人主动发出固定的问题："Hello, could you please press the elevator button for me?"，并通过语音合成系统播放出来。

## 文件说明

- `robot_voice_launcher/active_questioning_node.py`: 主动发问节点，负责发送固定文本到语音合成模块
- `launch/active_questioning.launch.py`: 单独启动主动发问功能的启动文件
- `launch/integrated_system.launch.py`: 集成启动文件，先启动主动发问，然后启动语音系统

## 使用方法

### 1. 单独启动主动发问功能

```bash
ros2 launch robot_voice_launcher active_questioning.launch.py
```

参数说明：
- `auto_start`: 是否在启动时自动发问（默认：true）
- `delay`: 启动后延迟发问的时间，单位秒（默认：2.0）

例如，设置5秒后自动发问：
```bash
ros2 launch robot_voice_launcher active_questioning.launch.py delay:=5.0
```

### 2. 启动集成系统（先主动发问，然后启动语音系统）

```bash
ros2 launch robot_voice_launcher integrated_system.launch.py
```

这将先启动主动发问节点，然后延迟10秒启动完整的语音系统。

### 3. 手动触发主动发问

如果已经启动了主动发问节点，可以通过服务调用手动触发发问：

```bash
ros2 service call /active_questioning_node/trigger_question std_srvs/srv/Empty
```

## 工作原理

1. 主动发问节点发布固定文本到`/llm_response`话题
2. `speech_generation_baidu`节点订阅该话题，将文本转换为语音
3. 合成的语音通过`/audio_generated`话题发送到`audio_play_python`节点
4. `audio_play_python`节点播放语音

## 注意事项

- 确保已安装所有依赖包
- 在使用集成启动文件时，系统会先发出问题，然后再启动完整的语音系统
- 如果需要修改问题文本，请编辑`active_questioning_node.py`文件中的`self.question_text`变量
