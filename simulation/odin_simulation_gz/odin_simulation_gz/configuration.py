import math
import re
from pathlib import Path
import yaml

def load_config(share, filename=''):
    result = yaml.safe_load((Path(share) / 'config/odin1.yaml').read_text())
    if filename:
        extra = yaml.safe_load(Path(filename).read_text())
        if not isinstance(extra, dict) or set(extra) - set(result):
            raise ValueError('Unknown sensor settings')
        result.update(extra)
    for k, v in result.items():
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0:
            raise ValueError(f'Invalid {k}: {v}')
        if k.endswith(('width', 'height')) and (not isinstance(v, int) or not 2 <= v <= 4096):
            raise ValueError(f'Invalid image dimension: {k}')
        if k.endswith('fov') and v >= 180:
            raise ValueError('Pinhole FOV must be below 180 degrees')
        if k.endswith('rate') and v > 1000:
            raise ValueError('Rate exceeds supported bound of 1000 Hz')
    if result['range_max'] <= result['range_min']:
        raise ValueError('Invalid range interval')
    if result['depth_width'] * result['depth_height'] > 1000000:
        raise ValueError('Too many rays')
    return {k: str(v) for k, v in result.items()}

def identifier(value):
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', value):
        raise ValueError(f'Invalid instance name: {value}')
    return value

def boolean(value):
    if value not in ('true', 'false'):
        raise ValueError('Expected true or false')
    return value == 'true'

def demo_isolation(domain, partition):
    """Explicit demo settings, never inferred from the caller's ROS/GZ env."""
    if not re.fullmatch(r'[0-9]+', domain) or not 0 <= int(domain) <= 232:
        raise ValueError('ros_domain_id must be an integer in [0, 232]')
    if not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.-]*', partition):
        raise ValueError('gz_partition must be a non-empty simple identifier')
    return str(int(domain)), partition
