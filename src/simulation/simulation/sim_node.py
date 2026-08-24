"""ROS 2 node wrapping :class:`simulation.world.World`.

Plays the world and the four hardware black boxes:
  publishes  /line_detection (grout_sim_msgs/LineDetection)  ~15 Hz
             /odom           (nav_msgs/Odometry)             ~50 Hz
             TF odom -> base_link
  subscribes /cmd_vel        (geometry_msgs/Twist)
             /grout_on       (std_msgs/Bool)

By design the simulation computes no score -- it only plays the world.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from grout_sim_msgs.msg import LineDetection

from .scenarios import get_scenario
from .world import World


def _yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class Simulation(Node):
    def __init__(self):
        super().__init__("simulation")
        self.declare_parameter("scenario", "happy_path")
        self.declare_parameter("seed", 0)
        self.declare_parameter("sim_rate", 100.0)
        self.declare_parameter("detection_rate", 15.0)
        self.declare_parameter("odom_rate", 50.0)
        self.declare_parameter("odom_noise_sigma", 0.002)

        name = self.get_parameter("scenario").get_parameter_value().string_value
        seed = int(self.get_parameter("seed").value)
        self.world = World(get_scenario(name), seed=seed)
        self._odom_sigma = float(self.get_parameter("odom_noise_sigma").value)
        self._odom_rng = np.random.default_rng(seed + 7)

        self._v = 0.0
        self._w = 0.0
        self._grout_on = False

        self.det_pub = self.create_publisher(LineDetection, "line_detection", 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.create_subscription(Twist, "cmd_vel", self._on_cmd, 10)
        self.create_subscription(Bool, "grout_on", self._on_grout, 10)
        self._tf = TransformBroadcaster(self)

        self._dt = 1.0 / float(self.get_parameter("sim_rate").value)
        self.create_timer(self._dt, self._step)
        self.create_timer(1.0 / float(self.get_parameter("detection_rate").value), self._pub_detection)
        self.create_timer(1.0 / float(self.get_parameter("odom_rate").value), self._pub_odom)
        self.get_logger().info(f"simulation playing scenario '{name}' (seed={seed})")

    def _on_cmd(self, msg: Twist):
        self._v = msg.linear.x
        self._w = msg.angular.z

    def _on_grout(self, msg: Bool):
        self._grout_on = msg.data

    def _step(self):
        self.world.step(self._dt, self._v, self._w)

    def _pub_detection(self):
        r = self.world.sense()
        m = LineDetection()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "extruder"
        m.extruder_distance = r.extruder_distance
        m.angle_difference = r.angle_difference
        m.valid = r.valid
        self.det_pub.publish(m)

    def _pub_odom(self):
        w = self.world
        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = _yaw_to_quat(w.theta)

        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = "odom"
        o.child_frame_id = "base_link"
        o.pose.pose.position.x = w.x + float(self._odom_rng.normal(0.0, self._odom_sigma))
        o.pose.pose.position.y = w.y + float(self._odom_rng.normal(0.0, self._odom_sigma))
        o.pose.pose.orientation.x = qx
        o.pose.pose.orientation.y = qy
        o.pose.pose.orientation.z = qz
        o.pose.pose.orientation.w = qw
        o.twist.twist.linear.x = w.last_v
        o.twist.twist.angular.z = w.last_w
        self.odom_pub.publish(o)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = w.x
        tf.transform.translation.y = w.y
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._tf.sendTransform(tf)


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
