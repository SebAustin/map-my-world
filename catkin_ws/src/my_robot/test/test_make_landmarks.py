"""Tests for scripts/make_landmarks.py. Run with: python3 -m pytest catkin_ws/src/my_robot/test"""

import os
import sys

import pytest

pytest.importorskip("PIL", reason="Pillow is only needed to regenerate the poster textures")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

import make_landmarks  # noqa: E402


def test_poster_is_deterministic_for_a_seed():
    a = make_landmarks.draw_poster("A", seed=0)
    b = make_landmarks.draw_poster("A", seed=0)
    assert a.tobytes() == b.tobytes()


def test_every_poster_looks_different():
    images = [make_landmarks.draw_poster(l, seed=i).tobytes()
              for i, l in enumerate(make_landmarks.POSTERS)]
    assert len(set(images)) == len(images)


def test_poster_is_busy_enough_to_give_features():
    img = make_landmarks.draw_poster("A", seed=0).convert("RGB")
    assert len(img.getcolors(maxcolors=1 << 16)) >= 6


def test_material_script_names_every_poster_and_texture():
    script = make_landmarks.material_script("AB")
    assert "material Landmark/PosterA" in script
    assert "texture poster_b.png" in script
    assert script.count("{") == script.count("}")


def test_write_model_creates_a_complete_gazebo_model(tmp_path):
    make_landmarks.write_model(str(tmp_path), letters="AB")
    assert (tmp_path / "model.config").is_file()
    assert (tmp_path / "model.sdf").is_file()
    assert (tmp_path / "materials" / "scripts" / "landmarks.material").is_file()
    for name in ("poster_a.png", "poster_b.png"):
        assert (tmp_path / "materials" / "textures" / name).is_file()
