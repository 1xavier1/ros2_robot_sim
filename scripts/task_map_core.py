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
    if task.get("type") != "taught_route":
        raise ValueError(f"unsupported task type: {task.get('type')}")
    if "route" not in task:
        raise ValueError(f"task {task_id} missing route")
    route = item_by_id(task_map["recorded_routes"], task["route"], "recorded_route")
    validate_route_path(route, task_id)
    if not route_allows_execution(task_map, route):
        raise ValueError(f"route {route['id']} violates motion profile")
    return [tuple(point["pose"]) for point in route["path"]]
