import math

MMWAVE_KEYS = ("mmwave_front", "mmwave_rear_left", "mmwave_rear_right")


def min_xy_distance(points):
    best = None
    for p in points:
        d = math.hypot(p[0], p[1])
        if best is None or d < best:
            best = d
    return best


def gated_map(cfg):
    return {
        key: bool(cfg[key].get("gated_in_push", False))
        for key in MMWAVE_KEYS
        if key in cfg
    }


def decide_estop(sensor_min, push_active, stop_distance, gated_in_push):
    for name, dmin in sensor_min.items():
        if dmin is None:
            continue
        if push_active and gated_in_push.get(name, False):
            continue
        if dmin <= stop_distance:
            return True
    return False
