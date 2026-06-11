#!/usr/bin/env python3
"""Pure helpers for taught task maps and P0 autonomous task navigation."""

import math
from pathlib import Path

import yaml


REQUIRED_TOP_LEVEL_KEYS = (
    "site",
    "maps",
    "motion_profiles",
    "regions",
    "waypoints",
    "recorded_routes",
    "tasks",
    "map_overlays",
)

MAPPING_TOP_LEVEL_KEYS = ("site", "maps", "map_overlays")
LIST_TOP_LEVEL_KEYS = (
    "motion_profiles",
    "regions",
    "waypoints",
    "recorded_routes",
    "tasks",
)


def load_task_map(path):
    with Path(path).open("r", encoding="utf-8") as stream:
        task_map = yaml.safe_load(stream)
    validate_task_map(task_map)
    return task_map


def validate_task_map(task_map):
    if not isinstance(task_map, dict):
        raise ValueError("task_map must be a mapping")
    missing = [key for key in REQUIRED_TOP_LEVEL_KEYS if key not in task_map]
    if missing:
        raise ValueError(f"task_map missing keys: {', '.join(missing)}")
    for key in MAPPING_TOP_LEVEL_KEYS:
        if not isinstance(task_map[key], dict):
            raise ValueError(f"task_map.{key} must be a mapping")
    for key in LIST_TOP_LEVEL_KEYS:
        if not isinstance(task_map[key], list):
            raise ValueError(f"task_map.{key} must be a list")
    if task_map["site"].get("map_frame") != "map":
        raise ValueError("P0 task_map site.map_frame must be map")
    return task_map


def point_on_segment(x, y, start, end, epsilon=1e-9):
    x1, y1 = start
    x2, y2 = end
    cross = (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)
    if abs(cross) > epsilon:
        return False
    return (
        min(x1, x2) - epsilon <= x <= max(x1, x2) + epsilon
        and min(y1, y2) - epsilon <= y <= max(y1, y2) + epsilon
    )


def point_in_polygon(x, y, polygon):
    inside = False
    j = len(polygon) - 1
    for i, point in enumerate(polygon):
        xi, yi = point
        xj, yj = polygon[j]
        if point_on_segment(x, y, (xi, yi), (xj, yj)):
            return True
        intersects = ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi
        )
        if intersects:
            inside = not inside
        j = i
    return inside


def validate_region_polygon(region, index):
    region_id = region.get("id", f"index {index}")
    if "polygon" not in region:
        raise ValueError(f"region {region_id} missing polygon")
    polygon = region["polygon"]
    if not isinstance(polygon, list) or len(polygon) < 3:
        raise ValueError(f"region {region_id} polygon must have at least 3 points")
    for point_index, point in enumerate(polygon):
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            raise ValueError(f"region {region_id} polygon point {point_index} invalid")
        if not all(
            isinstance(value, (int, float)) and not isinstance(value, bool)
            for value in point
        ):
            raise ValueError(f"region {region_id} polygon point {point_index} invalid")
    return polygon


def region_for_pose(task_map, x, y):
    for index, region in enumerate(task_map.get("regions", [])):
        polygon = validate_region_polygon(region, index)
        if point_in_polygon(x, y, polygon):
            return region
    return None


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def should_append_route_sample(samples, pose, min_distance, min_yaw_change):
    if not samples:
        return True
    previous = samples[-1]["pose"]
    distance = math.hypot(pose[0] - previous[0], pose[1] - previous[1])
    yaw_change = abs(wrap_angle(pose[2] - previous[2]))
    return distance >= min_distance or yaw_change >= min_yaw_change


def direction_from_linear_velocity(linear_x):
    return "reverse" if linear_x < 0.0 else "forward"


def xy_distance(pose_a, pose_b):
    return math.hypot(pose_a[0] - pose_b[0], pose_a[1] - pose_b[1])


def adapt_start_pose_for_ackermann(
    poses,
    current_pose=None,
    skip_distance=0.8,
    nearest_distance=2.0,
):
    if not poses or current_pose is None:
        return list(poses)
    adapted = [tuple(pose) for pose in poses]
    nearest_index = min(
        range(len(adapted)),
        key=lambda index: xy_distance(adapted[index], current_pose),
    )
    if xy_distance(adapted[nearest_index], current_pose) <= nearest_distance:
        adapted = adapted[nearest_index:]
    if len(adapted) > 1 and xy_distance(adapted[0], current_pose) <= skip_distance:
        adapted = adapted[1:]
    elif adapted:
        adapted[0] = (adapted[0][0], adapted[0][1], current_pose[2])
    return adapted


def motion_profiles_by_id(task_map):
    profiles = {}
    for profile in task_map.get("motion_profiles", []):
        profile_id = profile["id"]
        if profile_id in profiles:
            raise ValueError(f"duplicate motion_profile id: {profile_id}")
        profiles[profile_id] = profile
    return profiles


def route_allows_execution(task_map, route):
    profiles = motion_profiles_by_id(task_map)
    profile = profiles.get(route.get("motion_profile"))
    if profile is None:
        raise ValueError(f"unknown motion_profile: {route.get('motion_profile')}")
    if profile.get("allow_reverse", False):
        return True
    return all(point.get("direction", "forward") != "reverse" for point in route["path"])


def item_by_id(items, item_id, kind):
    for item in items:
        if item.get("id") == item_id:
            return item
    raise ValueError(f"unknown {kind}: {item_id}")


def validate_route_path(route, task_id):
    route_id = route.get("id")
    path = route.get("path")
    if not isinstance(path, list) or not path:
        raise ValueError(f"route {route_id} for task {task_id} must have non-empty path")
    for index, point in enumerate(path):
        if not isinstance(point, dict) or "pose" not in point:
            raise ValueError(
                f"route {route_id} for task {task_id} point {index} missing pose"
            )
        pose = point["pose"]
        if not isinstance(pose, (list, tuple)) or len(pose) != 3:
            raise ValueError(
                f"route {route_id} for task {task_id} point {index} pose "
                "must be 3 numeric values"
            )
        if not all(
            isinstance(value, (int, float)) and not isinstance(value, bool)
            for value in pose
        ):
            raise ValueError(
                f"route {route_id} for task {task_id} point {index} pose "
                "must be 3 numeric values"
            )


def goal_poses_for_task(task_map, task_id):
    task = item_by_id(task_map["tasks"], task_id, "task")
    if task.get("type") == "waypoint_sequence":
        waypoint_ids = task.get("waypoints", [])
        if not isinstance(waypoint_ids, list) or not waypoint_ids:
            raise ValueError(f"task {task_id} missing waypoints")
        poses = []
        for waypoint_id in waypoint_ids:
            waypoint = item_by_id(task_map["waypoints"], waypoint_id, "waypoint")
            pose = waypoint.get("pose")
            if not isinstance(pose, (list, tuple)) or len(pose) != 3:
                raise ValueError(f"waypoint {waypoint_id} pose must be 3 numeric values")
            if not all(
                isinstance(value, (int, float)) and not isinstance(value, bool)
                for value in pose
            ):
                raise ValueError(f"waypoint {waypoint_id} pose must be 3 numeric values")
            poses.append(tuple(pose))
        return poses
    if task.get("type") != "taught_route":
        raise ValueError(f"unsupported task type: {task.get('type')}")
    if "route" not in task:
        raise ValueError(f"task {task_id} missing route")
    route = item_by_id(task_map["recorded_routes"], task["route"], "recorded_route")
    validate_route_path(route, task_id)
    if not route_allows_execution(task_map, route):
        raise ValueError(f"route {route['id']} violates motion profile")
    return [tuple(point["pose"]) for point in route["path"]]
