#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tow_tractor.msg import ModelInfo
from std_msgs.msg import Float32

class DiffDriveController(Node):
    def __init__(self):
        super().__init__('diff_drive_controller')

        # Parameters
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('model_info_topic', '/info')
        self.declare_parameter('cmd_motor1_topic', '/cmd_motor1')
        self.declare_parameter('cmd_motor2_topic', '/cmd_motor2')

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        model_info_topic = self.get_parameter('model_info_topic').value
        cmd_motor1_topic = self.get_parameter('cmd_motor1_topic').value
        cmd_motor2_topic = self.get_parameter('cmd_motor2_topic').value

        # Publishers
        self.motor1_pub = self.create_publisher(Float32, cmd_motor1_topic, 10)
        self.motor2_pub = self.create_publisher(Float32, cmd_motor2_topic, 10)

        # Model parameters
        self.L = None
        self.r = None
        self.max_wheel_rpm = None

        # Subscriptions
        self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_callback, 10)
        qos_profile = rclpy.qos.QoSProfile(
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
        )
        self.create_subscription(ModelInfo, model_info_topic, self.model_info_callback, qos_profile)

    def model_info_callback(self, msg: ModelInfo):
        self.L = msg.wheel_distance
        self.r = msg.wheel_rad
        self.max_wheel_rpm = msg.max_rpm
        self.get_logger().info(f"Received model info: L={self.L}, r={self.r}, max_wheel_rpm={self.max_wheel_rpm}")

    def cmd_vel_callback(self, msg: Twist):
        """Handles the cmd_vel topic and converts into motor commands."""
        # --- Parameters (replace with ROS params as needed) ---
        if self.L is None or self.r is None or self.max_wheel_rpm is None:
            self.get_logger().warn('Model parameters not initialized yet. Skipping cmd_vel...')
            return

        # --- Extract velocities ---
        v = msg.linear.x
        omega = msg.angular.z

        # --- Differential drive kinematics ---
        v_r = v + (omega * self.L / 2.0)  # right wheel linear velocity (m/s)
        v_l = v - (omega * self.L / 2.0)  # left wheel linear velocity (m/s)

        # --- Convert to RPM ---
        rpm_r = (v_r / (2 * math.pi * self.r)) * 60.0
        rpm_l = (v_l / (2 * math.pi * self.r)) * 60.0

        # --- Clamp to max RPM ---
        rpm_r = max(min(rpm_r, self.max_wheel_rpm), -self.max_wheel_rpm)
        rpm_l = max(min(rpm_l, self.max_wheel_rpm), -self.max_wheel_rpm)

        # --- Store commands ---
        self.motor1_pub.publish(Float32(data=rpm_r))
        self.motor2_pub.publish(Float32(data=rpm_l))

def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
