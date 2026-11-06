"""Load explicit SITL profiles; pass one execution envelope to planner and sampler."""
import math
import yaml

# 锁定 PX4 版本的默认值；切换配置时覆盖上一次 SITL 留下的参数。
PX4_DEFAULT_LIMITS = {
    "MPC_XY_VEL_MAX": 12.0, "MPC_XY_CRUISE": 5.0,
    "MPC_ACC_HOR": 3.0, "MPC_ACC_HOR_MAX": 5.0,
    "MPC_JERK_AUTO": 4.0, "MPC_JERK_MAX": 8.0,
}

def load_flight_profile(root, name, config_root=None):
    if name == 'baseline':
        return dict(planner={}, execution={}, geometry=dict(tilt_degrees=15.,margin=.05),px4={**PX4_DEFAULT_LIMITS,"MPC_TILTMAX_AIR":45.0})
    if name != 'aggressive':
        raise ValueError('flight_profile must be baseline or aggressive')
    with flight_profile_path(root, name, config_root).open() as stream:
        data=yaml.safe_load(stream)
    p=data['planner'];e=data['execution'];g=data['geometry']
    for suffix in ('vel','acc','jer'):
        value=e['execution/max_'+suffix]
        if not math.isfinite(value) or value <= 0:
            raise ValueError('Execution limits must be finite and positive')
        if not 0 < p['optimization/max_'+suffix] <= value:
            raise ValueError('Optimization limits must fit inside the execution envelope')
    for suffix in ('vel','acc'):
        if p['manager/max_'+suffix] != p['optimization/max_'+suffix]:
            raise ValueError('Manager and optimizer limits must agree')
    if not 0 < p['fsm/planning_budget'] < p['fsm/replan_interval']:
        raise ValueError('Planning budget must be shorter than replan interval')
    if not 0 < g['tilt_degrees'] <= 45 or not math.isfinite(g['margin']) or g['margin'] < 0:
        raise ValueError('Invalid body envelope')
    if data['px4']['MPC_TILTMAX_AIR'] > g['tilt_degrees']:
        raise ValueError('PX4 tilt limit exceeds the collision envelope')
    data['px4'] = {**PX4_DEFAULT_LIMITS, **data['px4']}
    return data


def flight_profile_path(root, name, config_root=None):
    """飞行参数由 config/profiles 统一维护。"""
    if name == 'aggressive':
        return (config_root or root / 'config') / 'profiles/aggressive.yaml'
    raise ValueError('No profile file for ' + name)
