#!/usr/bin/env python3
"""
@file actuators_sender_node.py
@brief This node receives actuator and motor commands from ROS topics and sends them to the hardware via serial communication.

@details
ActuatorSenderNode is a ROS2 node that subscribes to motor and actuator command topics.
It receives commands for the right and left motors (as Float32) and for the actuator (as Bool),
then sends these commands to the hardware using the appropriate serial drivers.

- Subscribes to:
    - /cmd_motor1 (std_msgs/Float32): Command for the right motor
    - /cmd_motor2 (std_msgs/Float32): Command for the left motor
    - /cmd_actuator (std_msgs/Bool): Command for the actuator

- Sends commands to hardware using ActuatorsCommandsDriver and SerialSender.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from hardware.actuators_commands_driver import ActuatorsCommandsDriver
from hardware.serial_driver import SerialSender

class ActuatorSenderNode(Node):
    def __init__(self):
        super().__init__('actuators_sender_node')

        # --- Parameters - MOTORS ---
        self.declare_parameter('baudrate_motors', 115200)
        self.declare_parameter('channel_motors', '/dev/ttyACM0')
        self.declare_parameter('motors_msg_id', 0x10)
        self.declare_parameter('motors_msg_structure', 'ff')

        motors_baudrate = self.get_parameter('baudrate_motors').value
        motors_channel = self.get_parameter('channel_motors').value
        motors_msg_id = self.get_parameter('motors_msg_id').value
        motors_msg_structure = self.get_parameter('motors_msg_structure').value

        motors_sender_driver = SerialSender("motors_cmd", channel=motors_channel, msgID=motors_msg_id, msgIDLength=1, baudrate=motors_baudrate)
        self.motors_driver = ActuatorsCommandsDriver(
            driver=motors_sender_driver,
            acutators_commands_struct=motors_msg_structure,
            actuators_names=['right_motor_cmd', "left_motor_cmd"]
        )


        # --- Parameters - Actuator ---
        self.declare_parameter('baudrate_actuator', 115200)
        self.declare_parameter('channel_actuator', '/dev/ttyACM0')
        self.declare_parameter('actuator_msg_id', 0x11)
        self.declare_parameter('actuator_msg_structure', '?')

        actuator_baudrate = self.get_parameter('baudrate_actuator').value
        actuator_channel = self.get_parameter('channel_actuator').value
        actuator_msg_id = self.get_parameter('actuator_msg_id').value
        actuator_msg_structure = self.get_parameter('actuator_msg_structure').value

        actuator_sender_driver = SerialSender("actuator_cmd", channel=actuator_channel, msgID=actuator_msg_id, msgIDLength=1, baudrate=actuator_baudrate)
        self.actuator_driver = ActuatorsCommandsDriver(
            driver=actuator_sender_driver,
            acutators_commands_struct=actuator_msg_structure,
            actuators_names=['actuator_cmd']
        )


        # --- Parameters - Node ---
        self.declare_parameter('cmd_actuator_topic', '/cmd_actuator')
        self.declare_parameter('cmd_motor1_topic', '/cmd_motor1')
        self.declare_parameter('cmd_motor2_topic', '/cmd_motor2')
        self.declare_parameter('timer_period', 0.01)

        cmd_actuator_topic = self.get_parameter('cmd_actuator_topic').value
        cmd_motor1_topic = self.get_parameter('cmd_motor1_topic').value
        cmd_motor2_topic = self.get_parameter('cmd_motor2_topic').value
        timer_period = self.get_parameter('timer_period').value

        # --- initialize actuator commands ---
        self.cmd_right_motor = 0.0
        self.cmd_left_motor = 0.0
        self.cmd_actuator = False

        # --- subscriptions ---
        self.create_subscription(Float32, cmd_motor1_topic, self.cmd_motor1_callback, 10)
        self.create_subscription(Float32, cmd_motor2_topic, self.cmd_motor2_callback, 10)
        self.create_subscription(Bool, cmd_actuator_topic, self.cmd_actuator_callback, 10)

        self.create_timer(timer_period, self.send_commands)

    def cmd_motor1_callback(self, msg: Float32):
        self.cmd_right_motor = msg.data

    def cmd_motor2_callback(self, msg: Float32):
        self.cmd_left_motor = msg.data

    def cmd_actuator_callback(self, msg: Bool):
        self.cmd_actuator = msg.data

    def send_commands(self):
        self.motors_driver.send(
            self.cmd_right_motor,
            self.cmd_left_motor,
        )
        self.actuator_driver.send(
            self.cmd_actuator,
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