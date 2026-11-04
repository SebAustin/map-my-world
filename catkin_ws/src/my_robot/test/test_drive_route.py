"""Tests for scripts/drive_route.py. Run with: python3 -m pytest catkin_ws/src/my_robot/test"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

import drive_route  # noqa: E402

# Footprints of walls and Furniture in sebastien.world as (x_min, y_min, x_max, y_max).
OBSTACLES = [
    (-4.1, 2.925, 4.1, 3.1), (-4.1, -3.1, 4.1, -2.925), (-4.1, -3.1, -3.925, 3.1),
    (3.925, 0.6, 4.1, 3.1), (3.925, -3.1, 4.1, -0.6),
    (-0.075, 0.9, 0.075, 3.0), (-0.075, -3.0, 0.075, -0.9),
    (-3.3, 2.5, -1.7, 2.9), (-1.75, -1.75, -1.25, -1.25), (-3.9, -2.9, -2.7, -2.5),
    (-3.9, -2.5, -3.5, -1.5), (1.6, 1.2, 2.4, 2.0), (3.25, -2.7, 3.75, -1.7),
]
CLEARANCE = 0.3  # m: robot half-width plus a margin


def distance_to_box(px, py, box):
    x0, y0, x1, y1 = box
    return math.hypot(max(x0 - px, 0, px - x1), max(y0 - py, 0, py - y1))


def test_route_keeps_clear_of_walls_and_furniture():
    route = [(0.0, 0.0)] + drive_route.ROUTE + [drive_route.ROUTE[0]]
    for (ax, ay), (bx, by) in zip(route, route[1:]):
        for i in range(21):
            px, py = ax + (bx - ax) * i / 20, ay + (by - ay) * i / 20
            for box in OBSTACLES:
                assert distance_to_box(px, py, box) >= CLEARANCE, ((ax, ay), (bx, by), box)


def test_command_stops_at_the_goal():
    assert drive_route.command(1.0, 1.0, 0.0, (1.05, 1.0)) is None


def test_command_turns_in_place_when_facing_away():
    linear, angular = drive_route.command(0.0, 0.0, 0.0, (-2.0, 0.1))
    assert linear == 0.0 and angular != 0.0


def test_command_drives_forward_when_facing_the_goal():
    linear, angular = drive_route.command(0.0, 0.0, 0.0, (2.0, 0.0))
    assert linear == drive_route.MAX_SPEED and abs(angular) < 1e-9


def test_wrap_keeps_angles_in_range():
    assert abs(drive_route.wrap(3 * math.pi) - math.pi) < 1e-9 or abs(drive_route.wrap(3 * math.pi) + math.pi) < 1e-9
