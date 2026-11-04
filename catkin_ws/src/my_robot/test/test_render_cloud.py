"""Tests for scripts/render_cloud.py. Run with: python3 -m pytest catkin_ws/src/my_robot/test"""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

import render_cloud  # noqa: E402


def write_ply(path, points):
    header = ("ply\nformat binary_little_endian 1.0\nelement vertex %d\n"
              "property float x\nproperty float y\nproperty float z\n"
              "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n" % len(points))
    dtype = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("red", "u1"), ("green", "u1"), ("blue", "u1")])
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(np.array(points, dtype=dtype).tobytes())


def test_read_ply_returns_points_and_colours(tmp_path):
    path = tmp_path / "c.ply"
    write_ply(path, [(1, 2, 3, 255, 0, 0), (4, 5, 6, 0, 0, 255)])
    xyz, rgb = render_cloud.read_ply(str(path))
    assert xyz.tolist() == [[1, 2, 3], [4, 5, 6]]
    assert rgb.tolist() == [[1, 0, 0], [0, 0, 1]]


def test_read_ply_ignores_the_camera_element_pcl_appends(tmp_path):
    path = tmp_path / "pcl.ply"
    header = ("ply\nformat binary_little_endian 1.0\ncomment PCL generated\nelement vertex 1\n"
              "property float x\nproperty float y\nproperty float z\n"
              "property uchar red\nproperty uchar green\nproperty uchar blue\nproperty float nx\n"
              "element camera 1\nproperty float view_px\nproperty float view_py\nend_header\n")
    vertex = np.array([(1, 2, 3, 0, 255, 0, 0.5)],
                      dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("r", "u1"), ("g", "u1"), ("b", "u1"),
                             ("nx", "<f4")])
    camera = np.array([(9, 9)], dtype=[("a", "<f4"), ("b", "<f4")])
    path.write_bytes(header.encode("ascii") + vertex.tobytes() + camera.tobytes())
    xyz, rgb = render_cloud.read_ply(str(path))
    assert xyz.tolist() == [[1, 2, 3]] and rgb.tolist() == [[0, 1, 0]]


def test_read_ply_rejects_other_files(tmp_path):
    path = tmp_path / "x.ply"
    path.write_bytes(b"not a ply\n")
    with pytest.raises(ValueError):
        render_cloud.read_ply(str(path))


def test_select_drops_the_floor_and_caps_the_point_count():
    xyz = np.array([[0, 0, 0.0], [0, 0, 0.5], [0, 0, 0.6], [0, 0, 0.7]])
    rgb = np.zeros_like(xyz)
    kept, _ = render_cloud.select(xyz, rgb, limit=2)
    assert len(kept) == 2 and (kept[:, 2] > render_cloud.FLOOR_HEIGHT).all()


def test_select_drops_invalid_points():
    xyz = np.array([[0, 0, 0.5], [np.nan, 0, 0.5], [0, np.inf, 0.5]])
    kept, _ = render_cloud.select(xyz, np.zeros_like(xyz))
    assert kept.tolist() == [[0, 0, 0.5]]


def test_render_writes_an_image(tmp_path):
    pytest.importorskip("matplotlib")
    xyz = np.random.default_rng(0).uniform(0, 1, (100, 3))
    out = tmp_path / "map.png"
    render_cloud.render(xyz, xyz, str(out))
    assert out.stat().st_size > 0
