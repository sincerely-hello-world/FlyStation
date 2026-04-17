from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 参数文件的绝对路径
    # linktrack_param_file_uwb = "/home/orangepi/Desktop/UAV_CAR__Test/UAV_CAR/src/nlink_parser_ros2/params/linktrack_init_params.yaml"
    # linktrack_param_file_lidar = "/home/orangepi/Desktop/UAV_CAR__Test/UAV_CAR/src/bluesea-ros2/bluesea-ros2/params/uart_lidar.yaml"
    # 定义所有节点
    return LaunchDescription([

        Node(
            package='uav_car',
            executable='t265',
        ),

        # Node(
        #     package='uav_car',
        #     executable='camera',
        # ),
        # Node(
        #     package='uav_car',
        #     executable='image_processor',
        # ),

        Node(
            package='uav_car',
            executable='navigator',
        ),

        Node(
            package='uav_car',
            executable='uart',
        ),

        Node(
            package='uav_car',
            executable='qrcode',
        ),

        Node(
            package='uav_car',
            executable='qrcode2',
        ),
        
        Node(
            package='uav_car',
            executable='led',
        ),
    ])
