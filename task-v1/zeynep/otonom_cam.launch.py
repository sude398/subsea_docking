from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='my_pkg',
            executable='aruco_node_cam',
            name='aruco_detector',
            output='screen',
            emulate_tty=True,
        ),

        Node(
            package='my_pkg',
            executable='aruco_sub_node',
            name='aruco_subscriber',
            output='screen',
            emulate_tty=True,
        ),

    ])