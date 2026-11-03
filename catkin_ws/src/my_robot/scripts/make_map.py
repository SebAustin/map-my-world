#!/usr/bin/env python3
"""Build the Map (PGM + map_server YAML) straight from the World file.

Stands in for pgm_map_creator: every box and cylinder collision that crosses the
lidar's scan height becomes an occupied cell, everything else is free. Only
models written out in the world file count; <include>d models (ground plane,
sun) are skipped. Standard library only, so it runs on the VM's Python 3.8.

    python3 scripts/make_map.py worlds/sebastien.world maps/map
"""

import argparse
import math
import os
import xml.etree.ElementTree as ET
from typing import List, NamedTuple, Optional, Sequence, Tuple

Pose = Tuple[float, float, float, float]  # x, y, z, yaw
Bounds = Tuple[float, float, float, float]  # x_min, y_min, x_max, y_max
Grid = List[List[int]]

OCCUPIED = 0
FREE = 254

# Building is x -4..4, y -3..3; keep 1 m of margin so the east opening is mapped.
DEFAULT_BOUNDS: Bounds = (-5.0, -4.0, 5.0, 4.0)
DEFAULT_RESOLUTION = 0.05
# Height of the Hokuyo beams above the ground (see hokuyo_joint in my_robot.xacro).
DEFAULT_SCAN_HEIGHT = 0.27


class Shape(NamedTuple):
    kind: str  # "box" or "cylinder"
    x: float
    y: float
    yaw: float
    z_min: float
    z_max: float
    size_x: float = 0.0
    size_y: float = 0.0
    radius: float = 0.0

    def contains(self, px: float, py: float) -> bool:
        dx, dy = px - self.x, py - self.y
        if self.kind == "cylinder":
            return dx * dx + dy * dy <= self.radius * self.radius
        # Rotate the point into the box's own frame.
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy
        return abs(local_x) <= self.size_x / 2 and abs(local_y) <= self.size_y / 2


def parse_pose(text: Optional[str]) -> Pose:
    if not text or not text.strip():
        return (0.0, 0.0, 0.0, 0.0)
    values = [float(v) for v in text.split()]
    if len(values) != 6:
        raise ValueError("pose must have 6 values, got %r" % text)
    x, y, z, _roll, _pitch, yaw = values
    return (x, y, z, yaw)


def compose(parent: Pose, child: Pose) -> Pose:
    """Turn a child pose given in the parent frame into the frame the parent is given in.

    Roll and pitch are ignored: everything in this World stands upright.
    """
    px, py, pz, pyaw = parent
    cx, cy, cz, cyaw = child
    c, s = math.cos(pyaw), math.sin(pyaw)
    return (px + c * cx - s * cy, py + s * cx + c * cy, pz + cz, pyaw + cyaw)


def _pose_of(element: ET.Element) -> Pose:
    node = element.find("pose")
    return parse_pose(node.text if node is not None else None)


def _shape_from_geometry(geometry: ET.Element, pose: Pose) -> Optional[Shape]:
    x, y, z, yaw = pose
    box = geometry.find("box/size")
    if box is not None:
        sx, sy, sz = (float(v) for v in box.text.split())
        return Shape("box", x, y, yaw, z - sz / 2, z + sz / 2, size_x=sx, size_y=sy)
    cylinder = geometry.find("cylinder")
    if cylinder is not None:
        radius = float(cylinder.findtext("radius"))
        length = float(cylinder.findtext("length"))
        return Shape("cylinder", x, y, yaw, z - length / 2, z + length / 2, radius=radius)
    return None  # other geometry (mesh, sphere, plane) is not mapped


def load_shapes(world_path: str) -> List[Shape]:
    root = ET.parse(world_path).getroot()
    shapes = []
    for model in root.iter("model"):
        model_pose = _pose_of(model)
        for link in model.findall("link"):
            link_pose = compose(model_pose, _pose_of(link))
            for collision in link.findall("collision"):
                geometry = collision.find("geometry")
                if geometry is None:
                    continue
                shape = _shape_from_geometry(geometry, compose(link_pose, _pose_of(collision)))
                if shape is not None:
                    shapes.append(shape)
    return shapes


def rasterize(
    shapes: Sequence[Shape], bounds: Bounds, resolution: float, scan_height: float
) -> Grid:
    """Return image rows, top row first (max y), as map_server expects."""
    x_min, y_min, x_max, y_max = bounds
    width = int(round((x_max - x_min) / resolution))
    height = int(round((y_max - y_min) / resolution))
    seen = [s for s in shapes if s.z_min <= scan_height <= s.z_max]
    grid = []
    for row in range(height):
        cy = y_max - (row + 0.5) * resolution
        cells = []
        for col in range(width):
            cx = x_min + (col + 0.5) * resolution
            cells.append(OCCUPIED if any(s.contains(cx, cy) for s in seen) else FREE)
        grid.append(cells)
    return grid


def write_pgm(path: str, grid: Grid) -> None:
    height, width = len(grid), len(grid[0])
    with open(path, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (width, height))
        for row in grid:
            f.write(bytes(row))


def write_yaml(path: str, image: str, resolution: float, origin_xy: Tuple[float, float]) -> None:
    with open(path, "w") as f:
        f.write(
            "image: %s\n"
            "resolution: %s\n"
            "origin: [%s, %s, 0.0]\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n"
            "negate: 0\n" % (image, resolution, float(origin_xy[0]), float(origin_xy[1]))
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("world", help="Gazebo world file")
    parser.add_argument("output", help="output path without extension, e.g. maps/map")
    parser.add_argument("--resolution", type=float, default=DEFAULT_RESOLUTION)
    parser.add_argument("--scan-height", type=float, default=DEFAULT_SCAN_HEIGHT)
    parser.add_argument(
        "--bounds", type=float, nargs=4, default=DEFAULT_BOUNDS,
        metavar=("X_MIN", "Y_MIN", "X_MAX", "Y_MAX"),
    )
    args = parser.parse_args()

    shapes = load_shapes(args.world)
    if not shapes:
        parser.error("no box or cylinder collisions found in %s" % args.world)
    grid = rasterize(shapes, tuple(args.bounds), args.resolution, args.scan_height)
    write_pgm(args.output + ".pgm", grid)
    write_yaml(
        args.output + ".yaml",
        os.path.basename(args.output) + ".pgm",
        args.resolution,
        (args.bounds[0], args.bounds[1]),
    )
    print("wrote %s.pgm (%dx%d) from %d shapes" % (args.output, len(grid[0]), len(grid), len(shapes)))


if __name__ == "__main__":
    main()
