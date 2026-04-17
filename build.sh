#!/bin/bash

source /opt/ros/foxy/setup.sh

# 获取当前脚本所在目录的绝对路径（处理软链接等情况更健壮）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo '脚本所在目录: '$SCRIPT_DIR

## 获取上一层目录并赋值给一个变量
#PARENT_DIR=$(dirname "$SCRIPT_DIR") ## 父目录获取
#echo '脚本所在目录的父目录: '$PARENT_DIR

export ROS_LOG_DIR=$SCRIPT_DIR/run_log
echo '日志输出目录: '$ROS_LOG_DIR

# 初始化
echo 'ros2项目构建时所在目录: '$SCRIPT_DIR/UAV_CAR/
( cd $SCRIPT_DIR/UAV_CAR &&  colcon build --symlink-install )

# 构建产物
echo 'ros2项目构建产物: '$SCRIPT_DIR/UAV_CAR/install/
