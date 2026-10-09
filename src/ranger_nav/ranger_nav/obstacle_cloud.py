"""Filter navigation observations without inventing free-space rays."""
import copy
import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener


def transform_xyz(points, translation, quaternion):
    """Apply a target-from-source rigid transform to an Nx3 array."""
    if not np.isfinite(translation).all():
        raise ValueError('Invalid TF translation')
    x, y, z, w = np.asarray(quaternion, dtype=float)
    norm = x*x + y*y + z*z + w*w
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('Invalid TF quaternion')
    rotation = np.array([
        [1-2*(y*y+z*z)/norm, 2*(x*y-z*w)/norm, 2*(x*z+y*w)/norm],
        [2*(x*y+z*w)/norm, 1-2*(x*x+z*z)/norm, 2*(y*z-x*w)/norm],
        [2*(x*z-y*w)/norm, 2*(y*z+x*w)/norm, 1-2*(x*x+y*y)/norm],
    ])
    return points @ rotation.T + np.asarray(translation)


def filter_masks(points, min_height=0.08, max_height=1.60,
                 self_half_length=0.276, self_half_width=0.275,
                 self_height=1.50):
    """Return marking and clearing masks in the ground-projection frame."""
    values = (min_height, max_height, self_half_length, self_half_width, self_height)
    if (not all(math.isfinite(v) for v in values) or
            not 0 <= min_height < max_height or min(values[2:]) <= 0):
        raise ValueError('Invalid obstacle filter dimensions')
    finite = np.isfinite(points).all(axis=1)
    inside = ((np.abs(points[:, 0]) <= self_half_length) &
              (np.abs(points[:, 1]) <= self_half_width) &
              (points[:, 2] >= 0.) & (points[:, 2] <= self_height))
    clearing = finite & ~inside
    marking = clearing & (points[:, 2] >= min_height) & (points[:, 2] <= max_height)
    return marking, clearing


def cloud_arrays(msg):
    """Read XYZ and raw records, including organized/padded/endian clouds."""
    fields = {field.name: field for field in msg.fields}
    endian = '>' if msg.is_bigendian else '<'
    formats, offsets = [], []
    for name in ('x', 'y', 'z'):
        field = fields.get(name)
        if field is None or field.count != 1 or field.datatype not in (
                PointField.FLOAT32, PointField.FLOAT64):
            raise ValueError('PointCloud2 requires scalar floating-point XYZ')
        fmt = 'f4' if field.datatype == PointField.FLOAT32 else 'f8'
        if field.offset < 0 or field.offset + np.dtype(fmt).itemsize > msg.point_step:
            raise ValueError('XYZ field exceeds point_step')
        formats.append(endian + fmt)
        offsets.append(field.offset)
    if msg.width == 0 or msg.height == 0:
        return np.empty((0, 3)), np.empty((0, msg.point_step), dtype=np.uint8)
    if (msg.row_step < msg.width * msg.point_step or
            len(msg.data) < msg.row_step * msg.height):
        raise ValueError('Truncated PointCloud2 buffer')
    dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': formats,
                      'offsets': offsets, 'itemsize': msg.point_step})
    records = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                         strides=(msg.row_step, msg.point_step))
    xyz = np.column_stack([records[name].ravel() for name in ('x', 'y', 'z')])
    raw = np.ndarray((msg.height, msg.width, msg.point_step), dtype=np.uint8,
                     buffer=msg.data, strides=(msg.row_step, msg.point_step, 1))
    return xyz, raw.reshape(-1, msg.point_step)


def selected_cloud(msg, raw, mask):
    """Preserve source header and fields; output an unorganized subset."""
    output = PointCloud2()
    output.header = copy.deepcopy(msg.header)
    output.height = 1
    output.width = int(np.count_nonzero(mask))
    output.fields = copy.deepcopy(msg.fields)
    output.is_bigendian = msg.is_bigendian
    output.point_step = msg.point_step
    output.row_step = output.width * output.point_step
    output.data = raw[mask].tobytes()
    output.is_dense = True
    return output


class ObstacleCloud(Node):
    """Filter both observation streams using timestamped TF, failing closed."""

    def __init__(self):
        super().__init__('navigation_obstacle_cloud')
        defaults = {
            'input_topic': '/cloud_registered_body',
            'marking_topic': '/navigation/obstacle_points',
            'clearing_topic': '/navigation/clearing_points',
            'filter_frame': 'base_footprint',
            'min_height': 0.08, 'max_height': 1.60,
            'self_half_length': 0.276, 'self_half_width': 0.275,
            'self_height': 1.50, 'max_cloud_age': 0.5,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.geometry = {key: self.get_parameter(key).value for key in (
            'min_height', 'max_height', 'self_half_length', 'self_half_width',
            'self_height')}
        filter_masks(np.empty((0, 3)), **self.geometry)
        self.filter_frame = self.get_parameter('filter_frame').value
        self.max_cloud_age = self.get_parameter('max_cloud_age').value
        if not math.isfinite(self.max_cloud_age) or self.max_cloud_age <= 0:
            raise ValueError('max_cloud_age must be finite and positive')
        if any(not self.get_parameter(key).value for key in (
                'filter_frame', 'input_topic', 'marking_topic', 'clearing_topic')):
            raise ValueError('Frames and topics must not be empty')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.mark_pub = self.create_publisher(
            PointCloud2, self.get_parameter('marking_topic').value, qos)
        self.clear_pub = self.create_publisher(
            PointCloud2, self.get_parameter('clearing_topic').value, qos)
        self.subscription = self.create_subscription(
            PointCloud2, self.get_parameter('input_topic').value, self.on_cloud, qos)

    def on_cloud(self, msg):
        """Never substitute latest TF or an empty scan for an unavailable frame."""
        try:
            stamp = Time.from_msg(msg.header.stamp)
            age = (self.get_clock().now() - stamp).nanoseconds / 1e9
            if stamp.nanoseconds == 0 or not 0 <= age <= self.max_cloud_age:
                raise ValueError('Stale, future or zero-stamped cloud')
            transform = self.buffer.lookup_transform(
                self.filter_frame, msg.header.frame_id, stamp).transform
            xyz, raw = cloud_arrays(msg)
            t, q = transform.translation, transform.rotation
            points = transform_xyz(xyz, [t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
            marking, clearing = filter_masks(points, **self.geometry)
            self.mark_pub.publish(selected_cloud(msg, raw, marking))
            self.clear_pub.publish(selected_cloud(msg, raw, clearing))
        except (TransformException, ValueError, TypeError) as exc:
            self.get_logger().warning(f'Dropping navigation cloud: {exc}',
                                      throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ObstacleCloud()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
