"""Compose optional Nav2 parameters without duplicating the baseline file."""
import copy
import math


def merge_parameters(base, overrides):
    """Recursively merge mappings; lists and scalars replace baseline values."""
    result = copy.deepcopy(base)
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = merge_parameters(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def lidar_origin_arguments(config):
    """Read FAST-LIO's lidar-to-IMU transform (not its inverse)."""
    mapping = config['/**']['ros__parameters']['mapping']
    translation = mapping['extrinsic_T']
    rotation = mapping['extrinsic_R']
    if len(translation) != 3 or len(rotation) != 9:
        raise ValueError('Invalid FAST-LIO extrinsic dimensions')
    if not all(math.isfinite(v) for v in translation + rotation):
        raise ValueError('Non-finite FAST-LIO extrinsics')
    roll = math.atan2(rotation[7], rotation[8])
    pitch = math.atan2(-rotation[6], math.hypot(rotation[0], rotation[3]))
    yaw = math.atan2(rotation[3], rotation[0])
    args = []
    for key, value in zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'),
                          list(translation) + [roll, pitch, yaw]):
        args.extend(['--' + key, str(value)])
    return args + ['--frame-id', 'body', '--child-frame-id', 'navigation_lidar_origin']
