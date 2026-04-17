
#!/bin/bash
## 启动脚本start.sh

sleep 3

source /opt/ros/foxy/setup.bash
export PYTHONPATH="/usr/lib/python3/dist-packages/pyrealsense2:$PYTHONPATH"
#export LD_PRELOAD=/home/orangepi/.local/lib/python3.8/site-packages/torch/lib/../../torch.libs/libgomp-d22c30c5.so.1.0.0:$LD_PRELOAD


# 获取当前脚本所在目录的绝对路径（处理软链接等情况更健壮）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo '脚本所在目录: '$SCRIPT_DIR

## 获取上一层目录并赋值给一个变量
#PARENT_DIR=$(dirname "$SCRIPT_DIR") ## 父目录获取
#echo '脚本所在目录的父目录: '$PARENT_DIR

export ROS_LOG_DIR=$SCRIPT_DIR/run_log
echo '日志输出目录: '$ROS_LOG_DIR

# 初始化
echo 'ros2项目初始化bash所在路径: '$SCRIPT_DIR/UAV_CAR/install/setup.bash
source $SCRIPT_DIR/UAV_CAR/install/setup.bash # ros2项目启动bash所在路径


# 启动项目
echo 'ros2项目启动python所在路径: '$SCRIPT_DIR/UAV_CAR/install/uav_car_launch/share/uav_car_launch/launch/
echo 'ros2项目启动python源码所在路径: '$SCRIPT_DIR/UAV_CAR/src/uav_car_launch/launch/
# ros2 launch uav_car_launch uav_car_drone.launch.py

ros2 launch uav_car fly.launch.py
