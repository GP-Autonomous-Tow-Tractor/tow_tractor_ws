#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from tow_tractor.msg import ModelInfo
from hardware.actuators_commands_driver import ActuatorsCommandsDriver
from hardware.serial_driver import SerialSender

class ActuatorSenderNode(Node):
    def __init__(self):
        super().__init__('actuator_sender_node')

        # --- Connection parameters ---
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('channel', '/dev/ttyACM0')

        baudrate = self.get_parameter('baudrate').value
        channel = self.get_parameter('channel').value

        # --- Message parameters ---
        self.declare_parameter('actuators_msg_id', 0x10)
        self.declare_parameter('actuators_msg_structure', 'ff?')

        actuators_msg_id = self.get_parameter('actuators_msg_id').value
        actuators_msg_structure = self.get_parameter('actuators_msg_structure').value

        # --- Node parameters ---
        self.declare_parameter('cmd_actuator_topic', '/cmd_actuator')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('model_info_topic', '/info')
        self.declare_parameter('timer_period', 0.01)

        cmd_actuator_topic = self.get_parameter('cmd_actuator_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        model_info_topic = self.get_parameter('model_info_topic').value
        timer_period = self.get_parameter('timer_period').value
        # --- initialize actuator commands ---
        self.cmd_right_motor = 0.0
        self.cmd_left_motor = 0.0
        self.cmd_actuator = False

        self.L = None 
        self.r = None
        self.max_wheel_rpm = None

        # --- initialize actuator driver ---
        driver = SerialSender("actuators_cmd", channel=channel, msgID=actuators_msg_id, msgIDLength=1, baudrate=baudrate)
        self.actuator_driver = ActuatorsCommandsDriver(
            driver=driver,
            acutators_commands_struct=actuators_msg_structure,
            actuators_names=['right_motor_cmd', "left_motor_cmd", "actuator_cmd"]
        )

        # --- subscription ---
        self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_callback, 10)
        self.create_subscription(Bool, cmd_actuator_topic, self.cmd_actuator_callback, 10)

        qos_profile = rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE)
        self.create_subscription(ModelInfo, model_info_topic, self.model_info_callback, qos_profile)

        self.create_timer(timer_period, self.send_commands)

    def model_info_callback(self, msg: ModelInfo):
        """Handle received model information"""
        self.L = msg.wheel_base
        self.r = msg.wheel_rad
        self.max_wheel_rpm = msg.max_rpm

        # Log the received parameters
        self.get_logger().info(f"Received model info: L={self.L}, r={self.r}, max_wheel_rpm={self.max_wheel_rpm}")

    def cmd_vel_callback(self, msg: Twist):
        """Handles the cmd_vel topic and converts into motor commands."""
        # --- Parameters (replace with ROS params as needed) ---
        if self.L is None or self.r is None or self.max_wheel_rpm is None:
            self.get_logger().warn('Model parameters not initialized yet. Skipping cmd_vel...')
            return

        # --- Constants ---
        wheel_base = self.L
        wheel_radius = self.r
        max_rpm = self.max_wheel_rpm * 60 / (2 * math.pi * wheel_radius) # max wheel speed in RPM

        # --- Extract velocities ---
        v = msg.linear.x
        omega = msg.angular.z

        # --- Differential drive kinematics ---
        v_r = v + (omega * wheel_base / 2.0)  # right wheel linear velocity (m/s)
        v_l = v - (omega * wheel_base / 2.0)  # left wheel linear velocity (m/s)

        # --- Convert to RPM ---
        rpm_r = (v_r / (2 * math.pi * wheel_radius)) * 60.0
        rpm_l = (v_l / (2 * math.pi * wheel_radius)) * 60.0

        # --- Clamp to max RPM ---
        rpm_r = max(min(rpm_r, max_rpm), -max_rpm)
        rpm_l = max(min(rpm_l, max_rpm), -max_rpm)

        # --- Store commands ---
        self.cmd_right_motor = rpm_r
        self.cmd_left_motor = rpm_l

    def cmd_actuator_callback(self, msg: Bool):
        """Handle received actuator commands"""
        self.cmd_actuator = msg.data

    def send_commands(self):
        self.actuator_driver.send(
            self.cmd_right_motor,
            self.cmd_left_motor,
            self.cmd_actuator
        )

def main(args=None):
    rclpy.init(args=args)
    node = ActuatorSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()