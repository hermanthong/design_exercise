"""ROS 2 node wrapping :class:`simulation.world.World`.

Plays the world and the four hardware black boxes:
  publishes  /line_detection (msgs/LineDetection)  ~15 Hz
             /odom           (nav_msgs/Odometry)   ~50 Hz
             TF odom -> base_link
  subscribes /cmd_vel        (geometry_msgs/Twist)
             /extrude        (std_msgs/Bool)

The simulation plays the world using a simple kinematics model with noise added.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from msgs.msg import LineDetection

from .scenarios import get_scenario
from .world import World


def _yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class Simulation(Node):
    def __init__(self):
        super().__init__("simulation")
        self.declare_parameter("scenario", "happy_path")
        self.declare_parameter("sim_rate", 100.0)
        self.declare_parameter("detection_rate", 15.0)
        self.declare_parameter("odom_rate", 50.0)
        self.declare_parameter("odom_noise_sigma", 0.002)

        name = self.get_parameter("scenario").get_parameter_value().string_value
        self.world = World(get_scenario(name))
        self._odom_sigma = float(self.get_parameter("odom_noise_sigma").value)
        self._odom_rng = np.random.default_rng()

        self._linear_velocity = 0.0
        self._angular_velocity = 0.0
        self._extrude = False

        self.detection_publisher = self.create_publisher(LineDetection, "line_detection", 10)
        self.odom_publisher = self.create_publisher(Odometry, "odom", 10)
        self.create_subscription(Twist, "cmd_vel", self._cmd_vel_callback, 10)
        self.create_subscription(Bool, "extrude", self._extrude_callback, 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._dt = 1.0 / float(self.get_parameter("sim_rate").value)
        self.create_timer(self._dt, self._step)
        self.create_timer(1.0 / float(self.get_parameter("detection_rate").value), self._publish_detection)
        self.create_timer(1.0 / float(self.get_parameter("odom_rate").value), self._publish_odom)
        self.get_logger().info(f"simulation playing scenario '{name}'")

    def _cmd_vel_callback(self, msg: Twist):
        self._linear_velocity = msg.linear.x
        self._angular_velocity = msg.angular.z

    def _extrude_callback(self, msg: Bool):
        self._extrude = msg.data

    def _step(self):
        self.world.step(self._dt, self._linear_velocity, self._angular_velocity)

    def _publish_detection(self):
        reading = self.world.sense()
        detection = LineDetection()
        detection.header.stamp = self.get_clock().now().to_msg()
        detection.header.frame_id = "extruder"
        detection.extruder_distance = reading.extruder_distance
        detection.angle_difference = reading.angle_difference
        detection.valid = reading.valid
        self.detection_publisher.publish(detection)

    def _publish_odom(self):
        world = self.world
        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = _yaw_to_quat(world.theta)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = world.x + float(self._odom_rng.normal(0.0, self._odom_sigma))
        odom.pose.pose.position.y = world.y + float(self._odom_rng.normal(0.0, self._odom_sigma))
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = world.last_linear_velocity
        odom.twist.twist.angular.z = world.last_angular_velocity
        self.odom_publisher.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = world.x
        transform.transform.translation.y = world.y
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = Simulation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
