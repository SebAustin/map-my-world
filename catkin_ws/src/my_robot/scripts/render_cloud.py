#!/usr/bin/env python3
"""Render the 3D Map exported from a Database as a picture: an oblique view and a top view.

    rtabmap-export --output_dir export --output map --voxel 0.02 rtabmap.db
    python3 scripts/render_cloud.py export/map_cloud.ply docs/3d_map.png
"""

import argparse
import sys

import numpy as np

PLY_TYPES = {"float": "f4", "float32": "f4", "double": "f8", "uchar": "u1", "uint8": "u1",
             "int": "i4", "int32": "i4", "uint": "u4", "short": "i2", "ushort": "u2"}
FLOOR_HEIGHT = 0.03   # m: points below this are floor, left out so the walls and Landmarks stand out
MAX_POINTS = 200000   # plotting more is slow and adds nothing at this image size


def read_ply(path):
    """Return (xyz float array N x 3, rgb float array N x 3 in 0..1) from a binary little-endian PLY."""
    with open(path, "rb") as f:
        if f.readline().strip() != b"ply":
            raise ValueError("%s is not a PLY file" % path)
        fields, count, fmt, element = [], 0, None, None
        while True:
            line = f.readline().decode("ascii").strip()
            if line.startswith("format"):
                fmt = line.split()[1]
            elif line.startswith("element"):
                # PCL appends a "camera" element after the vertices; only the vertex layout matters here
                element = line.split()[1]
                if element == "vertex":
                    count = int(line.split()[2])
            elif line.startswith("property") and element == "vertex" and "list" not in line:
                _, kind, name = line.split()
                fields.append((name, "<" + PLY_TYPES[kind]))
            elif line == "end_header":
                break
        if fmt != "binary_little_endian":
            raise ValueError("only binary_little_endian PLY is supported, got %s" % fmt)
        data = np.fromfile(f, dtype=np.dtype(fields), count=count)
    xyz = np.stack([data["x"], data["y"], data["z"]], axis=1).astype(float)
    if "red" in data.dtype.names:
        rgb = np.stack([data["red"], data["green"], data["blue"]], axis=1) / 255.0
    else:
        rgb = np.full_like(xyz, 0.5)
    return xyz, rgb


def select(xyz, rgb, floor=FLOOR_HEIGHT, limit=MAX_POINTS, seed=0):
    keep = np.isfinite(xyz).all(axis=1)
    keep[keep] = xyz[keep, 2] > floor
    xyz, rgb = xyz[keep], rgb[keep]
    if len(xyz) > limit:
        idx = np.random.default_rng(seed).choice(len(xyz), limit, replace=False)
        xyz, rgb = xyz[idx], rgb[idx]
    return xyz, rgb


def render(xyz, rgb, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  registers the 3d projection on matplotlib < 3.2

    fig = plt.figure(figsize=(16, 7), facecolor="white")
    ax = fig.add_subplot(1, 2, 1, projection="3d")
    ax.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2], c=rgb, s=0.4, linewidths=0)
    ax.view_init(elev=35, azim=-60)
    span = np.ptp(xyz[:, :2], axis=0).max() / 2
    centre = xyz[:, :2].mean(axis=0)
    ax.set_xlim(centre[0] - span, centre[0] + span)
    ax.set_ylim(centre[1] - span, centre[1] + span)
    ax.set_zlim(0, 2 * span)
    ax.set_axis_off()
    ax.set_title("3D Map, oblique view")

    top = fig.add_subplot(1, 2, 2)
    order = np.argsort(xyz[:, 2])  # draw low points first so wall tops and posters show
    top.scatter(xyz[order, 0], xyz[order, 1], c=rgb[order], s=0.4, linewidths=0)
    top.set_aspect("equal")
    top.set_xlabel("x (m)")
    top.set_ylabel("y (m)")
    top.set_title("3D Map, top view")
    fig.tight_layout()
    fig.savefig(out, dpi=110)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("ply")
    parser.add_argument("out")
    args = parser.parse_args(argv)
    xyz, rgb = read_ply(args.ply)
    xyz, rgb = select(xyz, rgb)
    render(xyz, rgb, args.out)
    print("%s: %d points" % (args.out, len(xyz)))


if __name__ == "__main__":
    sys.exit(main())
