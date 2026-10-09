"""Fail-closed sensor freshness gate for Humble Collision Monitor output."""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener


class ObstacleVelocityGuard(Node):
    """Gate velocity on valid scan, filtered cloud, TF and command freshness."""

    def __init__(self):
        super().__init__('navigation_obstacle_guard')
        for name, default in (
                ('input_topic', '/navigation/collision_cmd_vel'),
                ('output_topic', '/cmd_vel_safe'),
                ('cloud_topic', '/navigation/obstacle_points'),
                ('scan_topic', '/scan'), ('base_frame', 'base_footprint'),
                ('timeout', 0.5)):
            self.declare_parameter(name, default)
        self.timeout = self.get_parameter('timeout').value
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('timeout must be finite and positive')
        self.base_frame = self.get_parameter('base_frame').value
        self.observations = {}
        self.last_command = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(
            Twist, self.get_parameter('output_topic').value, 1)
        self.create_subscription(PointCloud2, self.get_parameter('cloud_topic').value,
                                 lambda msg: self.observe('cloud', msg), qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value,
                                 lambda msg: self.observe('scan', msg), qos_profile_sensor_data)
        self.create_subscription(Twist, self.get_parameter('input_topic').value,
                                 self.on_command, 1)
        # Wall-clock watchdog must still run when /clock stops.
        from rclpy.clock import Clock, ClockType
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.05, self.watchdog, clock=self.wall_clock)

    def observe(self, key, msg):
        stamp = Time.from_msg(msg.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        try:
            if stamp.nanoseconds == 0 or not 0 <= age <= self.timeout:
                raise ValueError('invalid observation timestamp')
            self.buffer.lookup_transform(self.base_frame, msg.header.frame_id, stamp)
            self.observations[key] = (time.monotonic(), stamp)
        except (TransformException, ValueError):
            self.observations.pop(key, None)

    def healthy(self):
        now = time.monotonic()
        for key in ('cloud', 'scan'):
            observation = self.observations.get(key)
            if observation is None:
                return False
            received, stamp = observation
            age = (self.get_clock().now() - stamp).nanoseconds / 1e9
            if now - received > self.timeout or not 0 <= age <= self.timeout:
                return False
        return True

    def on_command(self, msg):
        values = (msg.linear.x, msg.linear.y, msg.linear.z,
                  msg.angular.x, msg.angular.y, msg.angular.z)
        self.last_command = time.monotonic()
        self.publisher.publish(msg if self.healthy() and all(map(math.isfinite, values))
                               else Twist())

    def watchdog(self):
        if (not self.healthy() or self.last_command is None or
                time.monotonic() - self.last_command > self.timeout):
            self.publisher.publish(Twist())
            self.get_logger().warning(
                'Voxel velocity gate closed: waiting for fresh cloud, scan, TF and command',
                throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ObstacleVelocityGuard()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.publisher.publish(Twist())
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
