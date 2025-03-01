#!/bin/bash

# 设置ROS2环境
source /opt/ros/iron/setup.bash
source /home/jay/microp_ws/install/setup.bash

# 默认参数
CHANNELS=2
SAMPLE_WIDTH=2
SAMPLE_RATE=16000
MAX_DURATION=60

# 解析命令行参数
while [[ $# -gt 0 ]]; do
  case $1 in
    --channels=*)
      CHANNELS="${1#*=}"
      shift
      ;;
    --sample-width=*)
      SAMPLE_WIDTH="${1#*=}"
      shift
      ;;
    --sample-rate=*)
      SAMPLE_RATE="${1#*=}"
      shift
      ;;
    --max-duration=*)
      MAX_DURATION="${1#*=}"
      shift
      ;;
    *)
      echo "未知参数: $1"
      exit 1
      ;;
  esac
done

echo "开始录音，参数: 声道=$CHANNELS, 采样宽度=$SAMPLE_WIDTH, 采样率=$SAMPLE_RATE, 最大时长=$MAX_DURATION"

# 运行音频记录器，传递参数
python3 /home/jay/microp_ws/src/audio_recorder.py --ros-args \
  -p channels:=$CHANNELS \
  -p sample_width:=$SAMPLE_WIDTH \
  -p sample_rate:=$SAMPLE_RATE \
  -p max_duration:=$MAX_DURATION
