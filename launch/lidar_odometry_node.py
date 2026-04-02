from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='lidar_dometry_node',
            executable='lidar_dometry_node',
            name='lidar_dometry_node',
            output='screen',
            parameters=['/home/robot/work_ws/src/lidar_dometry_node/config/lidar_dometry_node.yaml']
        )
    ])
