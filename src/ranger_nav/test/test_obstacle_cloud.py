"""Geometry regressions for full-body obstacle filtering."""
import numpy as np
import pytest

from ranger_nav.obstacle_cloud import filter_masks, transform_xyz


def test_binary_cloud_preserves_stamp_fields_and_padding():
    import struct
    from sensor_msgs.msg import PointCloud2, PointField
    from ranger_nav.obstacle_cloud import cloud_arrays, selected_cloud
    msg = PointCloud2()
    msg.header.frame_id = 'body'
    msg.header.stamp.sec = 123
    msg.height, msg.width = 2, 1
    msg.point_step, msg.row_step = 16, 20
    msg.is_bigendian = True
    msg.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
    msg.data = struct.pack('>ffff4xffff4x', 1., 0., 0., 7., 2., 0., 1., 9.)
    xyz, raw = cloud_arrays(msg)
    result = selected_cloud(msg, raw, np.array([False, True]))
    np.testing.assert_allclose(xyz, [[1., 0., 0.], [2., 0., 1.]])
    assert result.header == msg.header
    assert result.fields == msg.fields
    assert result.width == 1 and result.row_step == 16
    assert struct.unpack('>ffff', bytes(result.data)) == (2., 0., 1., 9.)


def test_overlay_does_not_change_global_obstacle_sources_or_baseline():
    from pathlib import Path
    import yaml
    from ranger_nav.voxel_config import merge_parameters, lidar_origin_arguments
    config = Path(__file__).parents[1] / 'config'
    base = yaml.safe_load((config / 'nav2_params_3dloc.yaml').read_text())
    overlay = yaml.safe_load((config / 'nav2_voxel_overrides.yaml').read_text())
    merged = merge_parameters(base, overlay)
    original = base['global_costmap']['global_costmap']['ros__parameters']
    changed = merged['global_costmap']['global_costmap']['ros__parameters']
    assert changed['obstacle_layer'] == original['obstacle_layer']
    assert base['local_costmap']['local_costmap']['ros__parameters']['plugins'][0] == 'obstacle_layer'
    fast = {'/**': {'ros__parameters': {'mapping': {
        'extrinsic_T': [-.011, -.02329, .04412],
        'extrinsic_R': [1., 0., 0., 0., 1., 0., 0., 0., 1.],
    }}}}
    args = lidar_origin_arguments(fast)
    assert args[args.index('--z') + 1] == '0.04412'


def test_ground_and_overhead_clear_but_do_not_mark():
    points = np.array([[1., 0., z] for z in (0., .079, .08, 1.6, 1.601)])
    marked, cleared = filter_masks(points)
    assert marked.tolist() == [False, False, True, True, False]
    assert cleared.all()


def test_self_and_invalid_returns_never_clear():
    points = np.array([[0., 0., 1.], [.276, .275, 1.5],
                       [.31, 0., .5], [np.nan, 0., .5], [1., np.inf, .5]])
    marked, cleared = filter_masks(points)
    assert marked.tolist() == [False, False, True, False, False]
    assert cleared.tolist() == marked.tolist()


def test_height_is_relative_to_footprint_not_map_zero():
    points = np.array([[1., 0., -.3], [1., 0., -.1]])
    transformed = transform_xyz(points, [0., 0., .3], [0., 0., 0., 1.])
    marked, cleared = filter_masks(transformed)
    assert marked.tolist() == [False, True]
    assert cleared.all()


def test_rotation_and_empty_cloud():
    result = transform_xyz(np.array([[1., 0., 0.]]), [0., 0., 0.],
                           [0., 0., 2 ** -.5, 2 ** -.5])
    np.testing.assert_allclose(result, [[0., 1., 0.]], atol=1e-10)
    marked, cleared = filter_masks(np.empty((0, 3)))
    assert marked.size == cleared.size == 0


def test_reject_invalid_filter_geometry():
    with pytest.raises(ValueError):
        filter_masks(np.empty((0, 3)), min_height=2., max_height=1.)


def test_launch_switch_composes_and_restores_baseline(monkeypatch, tmp_path):
    import importlib.util
    from pathlib import Path
    import yaml
    from launch import LaunchContext
    from launch.actions import SetLaunchConfiguration
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    package = Path(__file__).parents[1]
    launch_file = package / 'launch/navigation_3dloc.launch.py'
    spec = importlib.util.spec_from_file_location('voxel_launch_test', launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name: str(
        package if name == 'ranger_nav' else package.parent / 'FAST_LIO_ROS2'))
    context = LaunchContext()
    context.launch_configurations.update(use_voxel_obstacles='false', use_sim_time='false',
                                          voxel_visualization='false')
    actions = module._configure_voxel(context)
    assert len(actions) == 1
    actions[0].execute(context)
    assert context.launch_configurations['navigation_params'].endswith('nav2_params_3dloc.yaml')
    context.launch_configurations['use_voxel_obstacles'] = 'true'
    actions = module._configure_voxel(context)
    for action in actions:
        if isinstance(action, SetLaunchConfiguration):
            action.execute(context)
    generated = Path(context.launch_configurations['navigation_params'])
    try:
        params = yaml.safe_load(generated.read_text())
        assert params['collision_monitor']['ros__parameters']['cmd_vel_out_topic'] == (
            '/navigation/collision_cmd_vel')
        assert params['local_costmap']['local_costmap']['ros__parameters']['plugins'] == (
            ['voxel_layer', 'inflation_layer'])
    finally:
        generated.unlink()
