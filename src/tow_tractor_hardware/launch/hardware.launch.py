from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tow_tractor_hardware',
            executable='actuator_sender_node',
            name='actuator_sender_node',
            output='screen',
            parameters=[{'cmd_actuator_topic': '/cmd_actuator', 'cmd_vel_topic': '/cmd_vel', 'model_info_topic': '/info'}]
        ),
        Node(
            package='tow_tractor_control',
            executable='rear_wheel_steering_controller_node',
            name='rear_wheel_steering_controller_node',
            output='screen',
            parameters=[{'cmd_vel_topic': '/cmd_vel', 'model_info_topic': '/info', 'speed_cmd_topic': '/cmd_motor_speed', 'steer_cmd_topic': '/cmd_motor_pos'}]
        )
    ])
