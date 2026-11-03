"""Tests for scripts/make_map.py. Run with: python3 -m pytest catkin_ws/src/my_robot/test"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

import make_map  # noqa: E402

WORLD = os.path.join(os.path.dirname(__file__), "..", "worlds", "sebastien.world")

TINY_WORLD = """<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    <include><uri>model://ground_plane</uri></include>
    <model name="m">
      <pose>1 0 0 0 0 0</pose>
      <link name="wall">
        <pose>0 2 0 0 0 0</pose>
        <collision name="c">
          <pose>0 0 0.5 0 0 0</pose>
          <geometry><box><size>2 0.2 1</size></box></geometry>
        </collision>
      </link>
      <link name="post">
        <pose>-1 -1 0 0 0 0</pose>
        <collision name="c">
          <pose>0 0 0.5 0 0 0</pose>
          <geometry><cylinder><radius>0.3</radius><length>1</length></cylinder></geometry>
        </collision>
      </link>
      <link name="low_step">
        <pose>2 -2 0 0 0 0</pose>
        <collision name="c">
          <pose>0 0 0.05 0 0 0</pose>
          <geometry><box><size>0.5 0.5 0.1</size></box></geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
"""


@pytest.fixture
def tiny_world(tmp_path):
    path = tmp_path / "tiny.world"
    path.write_text(TINY_WORLD)
    return str(path)


def test_parse_pose_defaults_to_origin():
    assert make_map.parse_pose(None) == (0.0, 0.0, 0.0, 0.0)


def test_parse_pose_reads_x_y_z_yaw():
    assert make_map.parse_pose("1 2 3 0 0 0.5") == (1.0, 2.0, 3.0, 0.5)


def test_compose_rotates_child_by_parent_yaw():
    x, y, z, yaw = make_map.compose((1, 0, 0, math.pi / 2), (1, 0, 0.5, 0))
    assert (x, y, z, yaw) == pytest.approx((1, 1, 0.5, math.pi / 2))


def test_shapes_chain_model_link_and_collision_poses(tiny_world):
    shapes = {s.kind: s for s in make_map.load_shapes(tiny_world)}
    wall = make_map.load_shapes(tiny_world)[0]
    assert (wall.x, wall.y) == pytest.approx((1, 2))
    assert (wall.z_min, wall.z_max) == pytest.approx((0, 1))
    assert shapes["cylinder"].radius == pytest.approx(0.3)


def test_includes_like_ground_plane_are_ignored(tiny_world):
    assert len(make_map.load_shapes(tiny_world)) == 3


def test_box_contains_respects_yaw():
    rotated = make_map.Shape("box", 0, 0, math.pi / 2, 0, 1, 2.0, 0.2, 0.0)
    assert rotated.contains(0, 0.9)
    assert not rotated.contains(0.9, 0)


def test_cylinder_contains_by_radius():
    post = make_map.Shape("cylinder", 0, 0, 0, 0, 1, 0, 0, 0.3)
    assert post.contains(0.2, 0.2)
    assert not post.contains(0.3, 0.3)


def test_rasterize_marks_obstacles_at_scan_height_only(tiny_world):
    shapes = make_map.load_shapes(tiny_world)
    grid = make_map.rasterize(shapes, (-1, -3, 4, 3), 0.1, scan_height=0.27)
    width, height = 50, 60
    assert len(grid) == height and all(len(row) == width for row in grid)

    def cell(x, y):  # world point -> pixel value; row 0 is the top (max y)
        col = int((x + 1) / 0.1)
        row = height - 1 - int((y + 3) / 0.1)
        return grid[row][col]

    assert cell(1.0, 2.0) == make_map.OCCUPIED  # wall
    assert cell(0.0, -1.0) == make_map.OCCUPIED  # post
    assert cell(3.0, -2.0) == make_map.FREE  # low step is below the lidar
    assert cell(2.5, 0.0) == make_map.FREE


def test_write_pgm_and_yaml_round_trip(tmp_path):
    grid = [[make_map.FREE, make_map.OCCUPIED], [make_map.OCCUPIED, make_map.FREE]]
    pgm = tmp_path / "m.pgm"
    yaml = tmp_path / "m.yaml"
    make_map.write_pgm(str(pgm), grid)
    make_map.write_yaml(str(yaml), "m.pgm", 0.05, (-5.0, -4.0))

    data = pgm.read_bytes()
    assert data.startswith(b"P5\n2 2\n255\n")
    assert data[-4:] == bytes([make_map.FREE, make_map.OCCUPIED, make_map.OCCUPIED, make_map.FREE])
    text = yaml.read_text()
    assert "image: m.pgm" in text
    assert "origin: [-5.0, -4.0, 0.0]" in text
    assert "resolution: 0.05" in text


def test_real_world_has_walls_and_furniture_but_free_start_pose():
    shapes = make_map.load_shapes(WORLD)
    assert len(shapes) == 13  # 7 wall segments + 6 furniture pieces
    assert not any(s.contains(0.0, 0.0) for s in shapes)  # Start Pose is the open doorway
    assert any(s.contains(-1.5, -1.5) for s in shapes)  # pillar


def test_real_world_is_not_mirror_symmetric():
    shapes = make_map.load_shapes(WORLD)
    grid = make_map.rasterize(shapes, make_map.DEFAULT_BOUNDS, 0.05, scan_height=0.27)
    assert grid != [row[::-1] for row in grid]  # east-west mirror
    assert grid != grid[::-1]  # north-south mirror
    assert grid != [row[::-1] for row in grid[::-1]]  # 180-degree turn
