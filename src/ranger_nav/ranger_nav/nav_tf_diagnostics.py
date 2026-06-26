"""Small runtime checks for the KRT Nav2 TF/topic graph."""
import argparse
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


TF_CHECKS = [
    ('map', 'odom'),
    ('odom', 'camera_init'),
    ('camera_init', 'body'),
    ('body', 'base_footprint'),
    ('base_footprint', 'base_link'),
]

TOPICS = [
    '/odom',
    '/scan',
    '/cloud_registered_body',
    '/map',
    '/pcl_pose',
    '/alignment_status',
    '/initial_map',
    '/initialpose',
]


def _spin_for_tf(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--use-imu', action='store_true')
    args = parser.parse_args(argv)

    rclpy.init()
    node = Node('nav_tf_diagnostics')
    buffer = Buffer()
    TransformListener(buffer, node)
    errors = []

    try:
        _spin_for_tf(node, 2.0)

        node_names = set(node.get_node_names())
        has_amcl = any(name == 'amcl' or name.endswith('/amcl') for name in node_names)
        has_lidar_loc = any('pcl_localization' in name for name in node_names)
        if has_amcl and has_lidar_loc:
            errors.append(
                'AMCL and lidar localization are both running; only one may publish map->odom')

        topics = dict(node.get_topic_names_and_types())
        required_topics = TOPICS + (['/livox/imu'] if args.use_imu else [])
        for topic in required_topics:
            if topic in topics:
                print(f'OK topic {topic}')
            else:
                errors.append(f'missing topic {topic}')

        for parent, child in TF_CHECKS:
            if buffer.can_transform(parent, child, Time(), timeout=Duration(seconds=0.2)):
                print(f'OK TF {parent} -> {child}')
            else:
                errors.append(f'missing TF {parent} -> {child}')

        for error in errors:
            print(f'ERROR {error}', file=sys.stderr)
        return 1 if errors else 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
