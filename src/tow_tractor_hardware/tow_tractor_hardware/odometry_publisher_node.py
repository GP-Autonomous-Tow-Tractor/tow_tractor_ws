#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import numpy as np
from tow_tractor.msg import ModelInfo

class OdometryPublisherNode(Node):
    def __init__(self):
        super().__init__('odometry_publisher_node')

        # Declare parameters for topic names
        self.declare_parameter('right_motor_feedback_topic', '/feedback_right_motor_raw')
        self.declare_parameter('left_motor_feedback_topic', '/feedback_left_motor_raw')
        self.declare_parameter('odom_unfiltered_topic', '/odom_unfiltered')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('model_info_topic', '/info')

        # Get parameter values
        right_motor_feedback_topic = self.get_parameter('right_motor_feedback_topic').value
        left_motor_feedback_topic = self.get_parameter('left_motor_feedback_topic').value
        odom_unfiltered_topic = self.get_parameter('odom_unfiltered_topic').value
        joint_states_topic = self.get_parameter('joint_states_topic').value
        model_info_topic = self.get_parameter('model_info_topic').value

        # Set up subscribers
        self.create_subscription(Float32, right_motor_feedback_topic, self.right_motor_callback, 10)
        self.create_subscription(Float32, left_motor_feedback_topic, self.left_motor_callback, 10)
        qos_profile = rclpy.qos.QoSProfile(
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
        )
        self.create_subscription(ModelInfo, model_info_topic, self.model_info_callback, qos_profile)

        # Set up publishers
        self.odom_pub = self.create_publisher(Odometry, odom_unfiltered_topic, 10)
        self.joint_states_pub = self.create_publisher(JointState, joint_states_topic, 10)

        # Initialize state variables
        self.right_motor_rpm = 0.0
        self.left_motor_rpm = 0.0
        self.position = np.array([0.0, 0.0])  # Robot position [x, y]
        self.orientation = 0.0  # Robot orientation (theta) in radians
        self.left_wheel_angle = 0.0  # Accumulated angular position of left wheel
        self.right_wheel_angle = 0.0  # Accumulated angular position of right wheel

        # Model parameters to be set by ModelInfo message
        self.L = None  # Distance between wheels
        self.r = None  # Wheel radius

        # Time tracking for odometry updates
        self.last_time = self.get_clock().now()

        # Timer to publish odometry and joint states at 50 Hz
        self.create_timer(0.02, self.publish_odometry)

    def model_info_callback(self, msg: ModelInfo):
        """Callback to receive model parameters."""
        self.L = msg.wheel_distance
        self.r = msg.wheel_rad
        self.get_logger().info(f"Received model info: L={self.L}, r={self.r}")

    def right_motor_callback(self, msg):
        """Callback to update right motor RPM."""
        self.right_motor_rpm = msg.data

    def left_motor_callback(self, msg):
        """Callback to update left motor RPM."""
        self.left_motor_rpm = msg.data

    def publish_odometry(self):
        """Publish odometry and joint state messages."""
        if self.L is None or self.r is None:
            self.get_logger().warn('Model parameters not initialized yet. Skipping odometry...')
            return

        # Calculate time step
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            self.get_logger().warn('Non-positive time difference detected. Skipping odometry update...')
            return
        self.last_time = current_time

        # Calculate angular velocities of the wheels in rad/s
        angular_velocity_left = (self.left_motor_rpm * 2 * np.pi) / 60.0
        angular_velocity_right = (self.right_motor_rpm * 2 * np.pi) / 60.0

        # Update accumulated wheel angles by integrating angular velocities
        self.left_wheel_angle += angular_velocity_left * dt
        self.right_wheel_angle += angular_velocity_right * dt

        # Calculate linear velocities of the wheels in m/s
        v_r = angular_velocity_right * self.r
        v_l = angular_velocity_left * self.r

        # Compute robot's linear and angular velocities
        v = (v_r + v_l) / 2.0  # Linear velocity (m/s)
        omega = (v_r - v_l) / self.L  # Angular velocity (rad/s)

        # Update robot position and orientation
        self.orientation += omega * dt
        self.position[0] += v * np.cos(self.orientation) * dt  # x position
        self.position[1] += v * np.sin(self.orientation) * dt  # y position

        # Create and populate Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'  # Frame for pose
        odom_msg.child_frame_id = 'base_link'  # Frame for twist
        odom_msg.pose.pose.position.x = self.position[0]
        odom_msg.pose.pose.position.y = self.position[1]
        odom_msg.pose.pose.orientation.z = np.sin(self.orientation / 2)  # Quaternion z
        odom_msg.pose.pose.orientation.w = np.cos(self.orientation / 2)  # Quaternion w
        odom_msg.twist.twist.linear.x = v  # Linear velocity in x direction
        odom_msg.twist.twist.angular.z = omega  # Angular velocity around z axis
        self.odom_pub.publish(odom_msg)

        # Create and populate JointState message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = current_time.to_msg()
        joint_state_msg.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state_msg.position = [self.left_wheel_angle, self.right_wheel_angle]  # Angular positions
        joint_state_msg.velocity = [angular_velocity_left, angular_velocity_right]  # Angular velocities
        self.joint_states_pub.publish(joint_state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
