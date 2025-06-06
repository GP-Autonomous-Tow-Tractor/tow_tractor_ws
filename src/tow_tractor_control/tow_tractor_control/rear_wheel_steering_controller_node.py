#!/usr/bin/env python3
"""
ROS2 node: rear_wheel_steering_controller

Parameters (vehicle-specific, set via ROS2 parameters or defaults):
- cmd_vel_topic (string): name of Twist subscription topic  
- speed_cmd_topic (string): name of motor speed command publication topic  
- steer_cmd_topic (string): name of steering position command publication topic
- wheel_base (float): distance L between rear steering wheel and front axle [m]
- wheel_radius (float): radius of driving wheels [m]
- max_steering_angle (float): maximum rear-wheel steering angle [rad]
- mass (float): vehicle mass [kg] (for future use/dynamics) 

Subscriptions:
- cmd_vel_topic (geometry_msgs.msg.Twist)

Publications:
- speed_cmd_topic (std_msgs.msg.Float64): motor speed in rad/s
- steer_cmd_topic (std_msgs.msg.Float64): steering angle in radians

Transforms incoming Twist.linear.x and Twist.angular.z into individual commands
for rear-wheel steering and front-wheel drive.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from tow_tractor.msg import ModelInfo
from std_msgs.msg import Float64
import numpy as np

class RearWheelSteeringController(Node):
    def __init__(self):
        super().__init__('rear_wheel_steering_controller')

        # Topics
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('model_info_topic', '/info')
        self.declare_parameter('speed_cmd_topic', '/cmd_motor_speed')
        self.declare_parameter('steer_cmd_topic', '/cmd_motor_pos')

        # Retrive Parameters
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        model_info_topic = self.get_parameter('model_info_topic').value
        speed_topic = self.get_parameter('speed_cmd_topic').value
        steer_topic = self.get_parameter('steer_cmd_topic').value

        # Initialzation
        self.L = 0
        self.r = 0
        self.max_delta = 0
        self.max_wheel_speed = 0

        # Publishers
        self.speed_pub = self.create_publisher(Float64, speed_topic, 10)
        self.steer_pub = self.create_publisher(Float64, steer_topic, 10)

        # Subscribers
        qos_profile = rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE)
        self.create_subscription(ModelInfo, model_info_topic, self.model_info_callback, qos_profile)
        self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_callback, 10)
        self.get_logger().info(f'subscribed to {cmd_vel_topic}, publishing speed ->{speed_topic}, steer->{steer_topic}')

    def cmd_vel_callback(self, twist_msg: Twist):
        v = twist_msg.linear.x
        omega = -twist_msg.angular.z

        # v_min = 0.5
        # if v and abs(v) < v_min:
        #     v = v_min if v > 0 else -v_min

        if self.L == 0 or self.r == 0 or self.max_wheel_speed == 0:
            self.get_logger().warn('Model parameters not initialized yet. Skipping cmd_vel...')
            return

        # Calculate Steering Angle
        if abs(v) > 1e-3:
            sin_arg = omega * self.L / v
        else:
            sin_arg = np.sign(omega) * np.inf if omega != 0 else 0
        sin_arg = max(min(sin_arg, 1.0), -1)
        delta = np.arcsin(sin_arg)
        delta = max(min(delta, self.max_delta), -self.max_delta)

        if v < 0:
            delta *= -1

        # Calculate Motor Speed
        wheel_speed = 0.0
        if self.r > 1e-6:
            wheel_speed = v / self.r
        wheel_speed = max(min(wheel_speed, self.max_wheel_speed), -self.max_wheel_speed)

        # Convert to ROS msgs
        speed_msg = Float64()
        speed_msg.data = float(wheel_speed)
        steer_msg = Float64()
        steer_msg.data = float(delta)

        # Publish
        self.speed_pub.publish(speed_msg)
        self.steer_pub.publish(steer_msg)
        self.get_logger().info(f'cmd_vel: V={v:.2f} m/s, ω={omega:.2f} rad/s -> wheel ω={wheel_speed:.2f} rad/s, δ={delta:.2f} rad')

    def model_info_callback(self, model_info_msg: ModelInfo):
        self.L = model_info_msg.wheel_base
        self.r = model_info_msg.wheel_rad
        self.max_delta = model_info_msg.max_steering_angle
        self.max_wheel_speed = model_info_msg.max_rpm * 2 * np.pi / 60
        self.get_logger().info(f'Model loaded: L={self.L} m, r={self.r} m, max δ={self.max_delta} rad, max ω={self.max_wheel_speed:.2f} rad/s')

def main(args=None):
    rclpy.init(args=args)
    node = RearWheelSteeringController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
