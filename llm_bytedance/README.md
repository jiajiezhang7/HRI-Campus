# llm_bytedance

ROS2节点，用于与火山引擎大模型API交互。该节点接收来自语音识别的文本，将其发送给火山引擎的大模型API，然后将模型生成的响应发布到指定话题。

## 功能

- 订阅话题 `/speech_to_text`，接收语音识别的文本
- 调用火山引擎大模型API处理文本
- 将LLM生成的响应发布到话题 `/llm_response`

## 依赖

- ROS2
- Python 3
- `requests` Python包 (用于HTTP请求)

## 安装

```bash
# 克隆仓库到您的工作空间
cd ~/your_workspace/src
# 安装依赖
pip install requests
# 构建工作空间
cd ~/your_workspace
colcon build --packages-select llm_bytedance
# 刷新环境
source install/setup.bash
```

## 使用方法

### 设置API密钥

在使用前，请先设置火山引擎API密钥：

```bash
export ARK_API_KEY="您的API密钥"
```

### 启动节点

```bash
# 使用默认模型ID
ros2 launch llm_bytedance llm_bytedance.launch.py

# 或者指定模型ID
ros2 launch llm_bytedance llm_bytedance.launch.py model_id:=您的模型ID
```

### 测试节点

可以通过发布消息到 `/speech_to_text` 话题来测试：

```bash
ros2 topic pub /speech_to_text std_msgs/String "data: '请问今天天气怎么样？'" -1
```

然后检查 `/llm_response` 话题的输出：

```bash
ros2 topic echo /llm_response
```

## 配置项

- `ARK_API_KEY`: 火山引擎API密钥（通过环境变量设置）
- `ARK_MODEL_ID`: 火山引擎模型ID（可通过launch文件参数指定，默认值：`ep-20250213183336-ngs4d`）
- `system_prompt`: 系统提示词（可在代码中修改）

## 注意事项

- 请确保您已获取有效的火山引擎API密钥
- API调用可能会产生费用，请参考火山引擎官方文档了解定价
- 请勿将API密钥硬编码在代码中或公开分享
