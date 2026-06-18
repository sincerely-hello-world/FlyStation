from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    # 参数文件的绝对路径
    # linktrack_param_file_uwb = "/home/orangepi/Desktop/UAV_CAR__Test/UAV_CAR/src/nlink_parser_ros2/params/linktrack_init_params.yaml"
    # linktrack_param_file_lidar = "/home/orangepi/Desktop/UAV_CAR__Test/UAV_CAR/src/bluesea-ros2/bluesea-ros2/params/uart_lidar.yaml"
    # 定义所有节点

    node_Heartc = Node(
        package='uav_car',
        executable='heartc',
    )

    node_T265 = Node(
        package='uav_car',
        executable='t265',
    )

    node_navigator = Node(
        package='uav_car',
        executable='navigator',
    )

    node_uart = Node(
        package='uav_car',
        executable='uart',
    )

    node_led = Node(
        package='uav_car',
        executable='led',
    )

    node_servo = Node(
        package='uav_car',
        executable='servo',
    )

    node_camera = Node(
        package='uav_car',
        executable='camera',
    )

    # 注册事件处理器：当节点A启动完成后，启动
    reg_T265_start = RegisterEventHandler(
        OnProcessStart(
            target_action=node_T265, # T265节点先启动
            on_start=[node_uart,node_navigator]    # 启动之后，带动这些节点启动
        )
    )

    return LaunchDescription([
        
        node_T265,
        reg_T265_start,
        node_led,
        node_servo,
        node_camera,
        node_Heartc,
    ])
