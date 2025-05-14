#!/usr/bin/env python3
"""
@file sensor_receiver_node.py
@brief This node receives data from the IMU and encoders via serial communication and publishes it to ROS topics.
@SensorReceiverNode is a ROS2 node that interfaces with an IMU and encoders.
It reads data from the IMU and encoders, processes it, and publishes the data to
the appropriate ROS2 topics.

"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Imu

from hardware.imu_driver import IMUSerialDriver
from hardware.encoder_driver import EncoderSerialDriver
from hardware.serial_driver import SerialReceiver
class SensorReceiverNode(Node):
    def __init__(self):
        super().__init__('sensor_receiver_node')

        # Connection Parameter
        self.declare_parameter('baudrate_imu',115200)
        self.declare_parameter('channel_imu', '/dev/ttyACM0')

        self.declare_parameter('baudrate_encoders',115200)
        self.declare_parameter('channel_encoders', '/dev/ttyACM1')

        self.declare_parameter('baudrate_actuator_feedback',115200)
        self.declare_parameter('channel_actuator_feedback', '/dev/ttyACM2')

        baudrate_imu = self.get_parameter('baudrate_imu').value
        channel_imu = self.get_parameter('channel_imu').value
        baudrate_encoders = self.get_parameter('baudrate_encoders').value
        channel_encoders = self.get_parameter('channel_encoders').value
        baudrate_actuator_feedback = self.get_parameter('baudrate_actuator_feedback').value
        channel_actuator_feedback = self.get_parameter('channel_actuator_feedback').value

        # Messages Parmaters
        self.declare_parameter('imu_msg_id', 0x20)
        self.declare_parameter('encoders_msg_id', 0x30)
        self.declare_parameter('actuator_feedback_msg_id', 0x31)


        imu_msg_id = self.get_parameter('imu_msg_id').value
        encoder_msg_id = self.get_parameter('encoder_msg_id').value
        actuator_feedback_msg_id = self.get_parameter('actuator_feedback_msg_id').value

        # Node Parameters
        self.declare_parameter('timer_period', 0.002) # 2ms
        self.declare_parameter("/imu_topic", "/imu_raw")
        self.declare_parameter('/right_motor_feedback_topic', '/feedback_right_motor_raw')
        self.declare_parameter('/left_motor_feedback_topic', '/feedback_left_motor_raw')
        self.declare_parameter('/actuator_feedback_topic', '/actuator_status')

        imu_topic = self.get_parameter('/imu_topic').value
        right_motor_feedback_topic = self.get_parameter('/right_motor_feedback_topic').value
        left_motor_feedback_topic = self.get_parameter('/left_motor_feedback_topic').value
        actuator_feedback_topic = self.get_parameter('/actuator_feedback_topic').value
        timer_period = self.get_parameter('timer_period').value


        # Publishers
        self.imu_pub = self.create_publisher(Imu, imu_topic, 10)
        self.right_motor_pub = self.create_publisher(Float32, right_motor_feedback_topic, 10)  # Right motor topic
        self.left_motor_pub = self.create_publisher(Float32, left_motor_feedback_topic, 10)  # Left motor topic
        self.actuator_feedback_pub = self.create_publisher(Bool, actuator_feedback_topic, 10)  # Actuator feedback topic

        # Initialize Receivers Drivers
        self.imu = IMUSerialDriver(msgName="imu", channel=channel_imu, msgID=imu_msg_id, msgIDLength=1, baudrate=baudrate_imu)
        self.encoders = EncoderSerialDriver(msgName="encoders", channel=channel_encoders, msgID=encoder_msg_id, msgIDLength=1, baudrate=baudrate_encoders)
        self.actuator_feedback = SerialReceiver(msgName="actuator_feedback", channel=channel_actuator_feedback, msgID=actuator_feedback_msg_id, msgIDLength=1, baudrate=baudrate_actuator_feedback)

        # Timer for receiving & publishing data
        self.create_timer(timer_period, self.read_and_publish)

    def read_and_publish(self):
        # Read and publish IMU data
        try:
            imu_data = self.imu.receive()
            if imu_data:
                imu_msg = Imu()
                imu_msg.linear_acceleration.x = imu_data[0]
                imu_msg.linear_acceleration.y = imu_data[1]
                imu_msg.linear_acceleration.z = imu_data[2]
                imu_msg.angular_velocity.x = imu_data[3]
                imu_msg.angular_velocity.y = imu_data[4]
                imu_msg.angular_velocity.z = imu_data[5]
                self.imu_pub.publish(imu_msg)
        except Exception as e:
            self.get_logger().warn(f"IMU read error: {e}")

        # Read and publish encoder data
        try:
            encoders_data = self.encoders.receive(2, 4)  # Assuming this returns [right_motor_rpm, left_motor_rpm]
            if encoders_data:
                right_motor_msg = Float32()
                left_motor_msg = Float32()

                right_motor_msg.data = encoders_data[0]  # Right motor RPM
                left_motor_msg.data = encoders_data[1]  # Left motor RPM

                self.right_motor_pub.publish(right_motor_msg)  # Publish to /feedback_right_motor
                self.left_motor_pub.publish(left_motor_msg)  # Publish to /feedback_left_motor
        except Exception as e:
            self.get_logger().warn(f"Encoder read error: {e}")

        # Read and publish actuator feedback data
        try:
            actuator_feedback_data = self.actuator_feedback.receive()
            if actuator_feedback_data:
                actuator_feedback_data = bool.from_bytes(actuator_feedback_data, byteorder='little')
                actuator_feedback_msg = Bool()
                actuator_feedback_msg.data = actuator_feedback_data

                self.actuator_feedback_pub.publish(actuator_feedback_msg)  # Publish to /actuator_feedback
        except Exception as e:
            self.get_logger().warn(f"Actuator feedback read error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SensorReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
