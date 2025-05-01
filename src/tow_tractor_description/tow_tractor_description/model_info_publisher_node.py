#!/usr/bin/env python3
"""
ROS2 node: model_info_publisher

This node reads vehicle model parameters from the ROS2 Parameter Server (injected via launch/YAML)
and publishes a ModelInfo message on /<model_name>/info at 1 Hz.

Expected ROS2 parameters (all under ros__parameters):
  model_name:              string (e.g. "tow_tractor")

  # Physical Info
  moi:                     float64  # moment of inertia [kg·m^2]
  mass:                    float64  # total mass [kg]
  center_of_mass:          geometry_msgs/Point {x, y, z}

  # Limits
  max_rpm:                 float64  # maximum wheel RPM
  max_steering_angle:      float64  # max rear-wheel steering [rad]

  # Geometric Info
  wheel_base:              float64  # distance from rear steering wheel to front axle [m]
  wheel_distance:          float64  # track width between two front wheels [m]
  vehicle_length:          float64  # total length [m]
  vehicle_width:           float64  # total width [m]
"""
import rclpy
from rclpy.node import Node
from tow_tractor.msg import ModelInfo
from geometry_msgs.msg import Point

class ModelInfoPublisher(Node):
    def __init__(self):
        super().__init__('model_info_publisher')

        # Declare all expected parameters (defaults can be placeholders)
        self.declare_parameter('model_name', 'unknown_model')
        self.declare_parameter('moi', 0.0)
        self.declare_parameter('mass', 0.0)
        self.declare_parameter('center_of_mass.x', 0.0)
        self.declare_parameter('center_of_mass.y', 0.0)
        self.declare_parameter('center_of_mass.z', 0.0)
        self.declare_parameter('max_rpm', 0.0)
        self.declare_parameter('max_steering_angle', 0.0)
        self.declare_parameter('wheel_base', 0.0)
        self.declare_parameter('wheel_distance', 0.0)
        self.declare_parameter("wheel_rad", 0.0)
        self.declare_parameter('vehicle_length', 0.0)
        self.declare_parameter('vehicle_width', 0.0)

        # Fetch parameters
        model = self.get_parameter('model_name').value
        moi = self.get_parameter('moi').value
        mass = self.get_parameter('mass').value
        com = Point(
            x=self.get_parameter('center_of_mass.x').value,
            y=self.get_parameter('center_of_mass.y').value,
            z=self.get_parameter('center_of_mass.z').value
        )
        max_rpm = self.get_parameter('max_rpm').value
        max_steering_angle = self.get_parameter('max_steering_angle').value
        wheel_base = self.get_parameter('wheel_base').value
        wheel_distance = self.get_parameter('wheel_distance').value
        wheel_rad = self.get_parameter('wheel_rad').value
        vehicle_length = self.get_parameter('vehicle_length').value
        vehicle_width = self.get_parameter('vehicle_width').value

        # Prepare ModelInfo message
        self.info_msg = ModelInfo(
            model_name=model,
            moi=moi,
            mass=mass,
            center_of_mass=com,
            max_rpm=max_rpm,
            max_steering_angle=max_steering_angle,
            wheel_base=wheel_base,
            wheel_distance=wheel_distance,
            wheel_rad=wheel_rad,
            vehicle_length=vehicle_length,
            vehicle_width=vehicle_width
        )

        topic = f'/{model}/info'
        self.publisher = self.create_publisher(
            ModelInfo, 
            topic, 
            qos_profile=rclpy.qos.QoSProfile(
                depth=1, 
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
            )
        )
        
        # Publish immediately
        self.publish_model_info()
        self.get_logger().info(f'Publishing ModelInfo on topic: {topic}')


    def publish_model_info(self):
        self.publisher.publish(self.info_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ModelInfoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
