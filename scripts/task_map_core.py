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
