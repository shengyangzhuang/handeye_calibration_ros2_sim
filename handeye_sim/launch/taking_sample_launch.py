from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='handeye_sim',
            executable='robot',
            name='robot_state_estimation'
        ),
        Node(
            package='handeye_sim',
            executable='aruco',
            name='aruco_estimation'
        ),
    ])