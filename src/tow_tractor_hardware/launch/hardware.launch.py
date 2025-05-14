from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tow_tractor_hardware',
            executable='sensor_receiver_node',
            name='sensor_receiver_node',
            output='screen',
        ),
        Node(
            package='tow_tractor_hardware',
            executable='actuator_sender_node',
            name='actuator_sender_node',
            output='screen',
        ),
    ])
