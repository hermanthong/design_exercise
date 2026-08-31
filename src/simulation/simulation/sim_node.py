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
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from msgs.msg import LineDetection

from .geometry import angle_wrap
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

        name = self.get_parameter("scenario").get_parameter_value().string_value
        scenario = get_scenario(name)
        self.world = World(scenario)
        self._camera_delay = float(scenario.camera_delay)
        self._detection_history = deque()  # (time_ns, lateral_error, heading_error, valid)
        self._odom_drift_rate = float(scenario.odom_drift_rate)
        self._odom_delay = float(scenario.odom_delay)
        self._odom_rng = np.random.default_rng()

        # Odometry estimate, integrated from the true motion with accumulating drift.
        # It starts at the true launch pose and only changes while the robot moves.
        self._odom_x = self.world.x
        self._odom_y = self.world.y
        self._odom_theta = self.world.theta
        self._prev_x = self.world.x
        self._prev_y = self.world.y
        self._prev_theta = self.world.theta
        self._odom_history = deque()  # (time_ns, x, y, theta, v, w) for latency

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
        now = self.get_clock().now()
        reading = self.world.sense()

        # Latency: buffer readings and publish the one from camera_delay seconds ago.
        self._detection_history.append(
            (now.nanoseconds, reading.lateral_error, reading.heading_error, reading.valid)
        )
        delay_ns = int(self._camera_delay * 1e9)
        while len(self._detection_history) > 1 and now.nanoseconds - self._detection_history[1][0] >= delay_ns:
            self._detection_history.popleft()
        _, lateral_error, heading_error, valid = self._detection_history[0]

        detection = LineDetection()
        detection.header.stamp = now.to_msg()
        detection.header.frame_id = "extruder"
        detection.lateral_error = lateral_error
        detection.heading_error = heading_error
        detection.valid = valid
        self.detection_publisher.publish(detection)

    def _publish_odom(self):
        world = self.world
        now = self.get_clock().now()
        stamp = now.to_msg()

        dx = world.x - self._prev_x
        dy = world.y - self._prev_y
        dyaw = angle_wrap(world.theta - self._prev_theta)
        self._prev_x, self._prev_y, self._prev_theta = world.x, world.y, world.theta

        # add noise proportional to the true incremental motion
        ds = dx * math.cos(world.theta) + dy * math.sin(world.theta)  # signed forward distance
        motion = abs(ds) + abs(dyaw)
        ds += float(self._odom_rng.normal(0.0, self._odom_drift_rate * abs(ds)))
        dyaw += float(self._odom_rng.normal(0.0, self._odom_drift_rate * motion))
        self._odom_theta = angle_wrap(self._odom_theta + dyaw)
        self._odom_x += ds * math.cos(self._odom_theta)
        self._odom_y += ds * math.sin(self._odom_theta)

        # add delay
        self._odom_history.append(
            (now.nanoseconds, self._odom_x, self._odom_y, self._odom_theta,
             world.last_linear_velocity, world.last_angular_velocity)
        )
        delay_ns = int(self._odom_delay * 1e9)
        while len(self._odom_history) > 1 and now.nanoseconds - self._odom_history[1][0] >= delay_ns:
            self._odom_history.popleft()
        _, ox, oy, otheta, ov, ow = self._odom_history[0]

        oqx, oqy, oqz, oqw = _yaw_to_quat(otheta)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = ox
        odom.pose.pose.position.y = oy
        odom.pose.pose.orientation.x = oqx
        odom.pose.pose.orientation.y = oqy
        odom.pose.pose.orientation.z = oqz
        odom.pose.pose.orientation.w = oqw
        odom.twist.twist.linear.x = ov
        odom.twist.twist.angular.z = ow
        self.odom_publisher.publish(odom)

        # TF odom -> base_link stays as ground truth
        tqx, tqy, tqz, tqw = _yaw_to_quat(world.theta)
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = world.x
        transform.transform.translation.y = world.y
        transform.transform.rotation.x = tqx
        transform.transform.rotation.y = tqy
        transform.transform.rotation.z = tqz
        transform.transform.rotation.w = tqw
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
