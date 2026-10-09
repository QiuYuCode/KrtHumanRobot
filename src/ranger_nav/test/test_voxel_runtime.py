"""Opt-in real Nav2 smoke test, strictly isolated from the robot DDS domain.

Run with KRT_VOXEL_RUNTIME_TEST=1 ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1.
All velocity output is remapped to a test-only topic; no hardware nodes run.
"""
import os
import copy
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest
import yaml

pytestmark = pytest.mark.skipif(
    os.environ.get('KRT_VOXEL_RUNTIME_TEST') != '1', reason='opt-in isolated ROS smoke test')


def test_real_voxel_and_collision_monitor(tmp_path):
    assert os.environ.get('ROS_DOMAIN_ID') == '91'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    import rclpy
    from geometry_msgs.msg import TransformStamped, Twist
    from lifecycle_msgs.srv import ChangeState
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import LaserScan, PointCloud2
    from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
    from std_msgs.msg import Header
    from tf2_ros import StaticTransformBroadcaster
    from rclpy.qos import qos_profile_sensor_data
    from ranger_nav.voxel_config import merge_parameters

    config = Path(__file__).parents[1] / 'config'
    base = yaml.safe_load((config / 'nav2_params_3dloc.yaml').read_text())
    overlay = yaml.safe_load((config / 'nav2_voxel_overrides.yaml').read_text())
    params = merge_parameters(base, overlay)
    # Standalone Costmap2DROS hardcodes /costmap/costmap in Humble.
    params['costmap'] = {'costmap': copy.deepcopy(params['local_costmap']['local_costmap'])}
    path = tmp_path / 'params.yaml'
    path.write_text(yaml.safe_dump(params))
    processes, logs = [], []
    rclpy.init()
    node = rclpy.create_node('voxel_test_observer')
    try:
        def start(command):
            log = (tmp_path / f'node_{len(logs)}.log').open('w+')
            logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=log))

        ros = ['--ros-args', '--params-file', str(path)]
        start([sys.executable, '-c', 'from ranger_nav.obstacle_cloud import main; main()'] + ros)
        start(['/opt/ros/humble/lib/nav2_costmap_2d/nav2_costmap_2d'] + ros +
              ['-r', '__node:=local_costmap', '-r', '__ns:=/local_costmap'])
        start(['/opt/ros/humble/lib/nav2_collision_monitor/collision_monitor'] + ros +
              ['-r', '/cmd_vel:=/voxel_test/input', '-r', '/cmd_vel_safe:=/voxel_test/output'])
        start([sys.executable, '-c', 'from ranger_nav.obstacle_velocity_guard import main; main()'] +
              ['--ros-args', '-r', '/cmd_vel_safe:=/voxel_test/output'])
        start(['/opt/ros/humble/lib/nav2_costmap_2d/nav2_costmap_2d_cloud',
               '--ros-args', '-r', '__ns:=/costmap'])
        broadcaster = StaticTransformBroadcaster(node)
        transforms = []
        for parent, child, xyz in (
                ('odom', 'body', (0., 0., 0.)),
                ('body', 'base_footprint', (-.2, 0., -.3)),
                ('body', 'navigation_lidar_origin', (-.011, -.02329, .04412))):
            tf = TransformStamped()
            tf.header.frame_id, tf.child_frame_id = parent, child
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = xyz
            tf.transform.rotation.w = 1.
            transforms.append(tf)
        broadcaster.sendTransform(transforms)
        clouds = node.create_publisher(PointCloud2, '/cloud_registered_body', qos_profile_sensor_data)
        scans = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        commands = node.create_publisher(Twist, '/voxel_test/input', 10)
        received, costs, velocities = [], [], []
        voxels = []
        node.create_subscription(PointCloud2, '/costmap/voxel_marked_cloud', voxels.append,
                                 qos_profile_sensor_data)
        node.create_subscription(PointCloud2, '/navigation/obstacle_points', received.append,
                                 qos_profile_sensor_data)
        node.create_subscription(OccupancyGrid, '/costmap/costmap', costs.append, 10)
        node.create_subscription(Twist, '/voxel_test/output', velocities.append, 10)

        def spin_until(predicate, timeout=8.):
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.05)
                assert all(p.poll() is None for p in processes), 'ROS process exited'
                if predicate():
                    return
            pytest.fail('Timed out waiting for ROS state; inspect node logs in ' + str(tmp_path))

        for target in ('/costmap/costmap', '/collision_monitor'):
            client = node.create_client(ChangeState, target + '/change_state')
            assert client.wait_for_service(timeout_sec=15.), target
            for transition in (1, 3):
                req = ChangeState.Request()
                req.transition.id = transition
                future = client.call_async(req)
                spin_until(future.done)
                assert future.result().success, target

        def feed(points, duration=1.5, frame='body', cloud_enabled=True,
                 scan_enabled=True, command_enabled=True):
            end = time.monotonic() + duration
            while time.monotonic() < end:
                header = Header(frame_id=frame, stamp=node.get_clock().now().to_msg())
                if cloud_enabled:
                    clouds.publish(create_cloud_xyz32(header, points))
                scan = LaserScan(header=Header(frame_id='body', stamp=header.stamp))
                scan.angle_min, scan.angle_max, scan.angle_increment = -1., 1., 1.
                scan.range_min, scan.range_max = .1, 30.
                scan.ranges = [float('inf')] * 3
                if scan_enabled:
                    scans.publish(scan)
                twist = Twist()
                twist.linear.x = .2
                if command_enabled:
                    commands.publish(twist)
                for _ in range(8):
                    rclpy.spin_once(node, timeout_sec=.005)
                time.sleep(.025)

        # Ground is at odom z=-0.3, not z=0; it must never become an obstacle.
        feed([[.8, .1, -.3]])
        spin_until(lambda: received and costs and velocities)
        assert received[-1].width == 0
        assert max(costs[-1].data) < 100
        assert velocities[-1].linear.x == pytest.approx(.2)

        # A high obstacle above the old 1.2 m slice must mark a costmap cell.
        obstacle = [.8, .1, 1.1]  # 1.4 m above the floor
        feed([obstacle])
        assert received[-1].width == 1
        assert 100 in costs[-1].data
        spin_until(lambda: voxels and voxels[-1].width > 0)
        # Same ray, observed beyond the previous obstacle: clear that voxel.
        origin = [-.011, -.02329, .04412]
        endpoint = [o + 1.6*(p-o) for o, p in zip(origin, obstacle)]
        feed([endpoint], duration=2.)
        assert max(costs[-1].data) < 100

        # Four distinct table-edge returns, at footprint x=0.4 / 0.65.
        for x, speed in ((.45, .1), (.2, 0.)):
            feed([[x, y, .5] for y in (-.04, -.02, .02, .04)])
            assert velocities[-1].linear.x == pytest.approx(speed)
        feed([[.8, .1, -.3]])
        assert velocities[-1].linear.x == pytest.approx(.2)
        # Missing TF must not produce empty 'clear' scans; monitoring stops.
        missing_since = node.get_clock().now().nanoseconds
        feed([[.8, .1, .5]], frame='missing_sensor_frame', duration=.9)
        assert all(rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds < missing_since
                   for msg in received)
        assert velocities[-1].linear.x == 0.
        feed([[.8, .1, -.3]])
        assert velocities[-1].linear.x == pytest.approx(.2)
        feed([], cloud_enabled=False, duration=.9)
        assert velocities[-1].linear.x == 0.
        feed([[.8, .1, -.3]])
        assert velocities[-1].linear.x == pytest.approx(.2)
        feed([[.8, .1, -.3]], scan_enabled=False, duration=.9)
        assert velocities[-1].linear.x == 0.
        feed([[.8, .1, -.3]])
        assert velocities[-1].linear.x == pytest.approx(.2)
        feed([[.8, .1, -.3]], command_enabled=False, duration=.9)
        assert velocities[-1].linear.x == 0.
    finally:
        for process in processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=5.)
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()
