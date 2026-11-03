#!/usr/bin/env bash
# Static checks that run without Gazebo: XML is well formed, the Map is in sync with the World,
# the map YAML matches its PGM, and the Start Pose agrees with the AMCL initial pose.
# On the ROS VM it also asks roslaunch to resolve both launch files.
set -euo pipefail

cd "$(dirname "$0")"
PKG=catkin_ws/src/my_robot
fail=0
ok()  { echo "  ok    $1"; }
bad() { echo "  FAIL  $1"; fail=1; }

echo "XML well-formedness"
for f in "$PKG"/launch/*.launch "$PKG"/worlds/*.world "$PKG"/urdf/*.gazebo "$PKG"/urdf/*.xacro "$PKG"/package.xml; do
  if xmllint --noout "$f" 2>/dev/null; then ok "$f"; else bad "$f"; fi
done

echo "Map generator tests"
if python3 -m pytest -q "$PKG/test" >/dev/null 2>&1; then ok "pytest"; else bad "pytest (run: python3 -m pytest $PKG/test)"; fi

echo "Map matches World and YAML"
python3 - "$PKG" <<'EOF' || fail=1
import os, re, sys, tempfile
pkg = sys.argv[1]
sys.path.insert(0, os.path.join(pkg, "scripts"))
import make_map

problems = []
with open(os.path.join(pkg, "maps", "map.pgm"), "rb") as f:
    data = f.read()
width, height = map(int, data.split(b"\n", 2)[1].split())  # header: P5 / W H / 255
yaml = open(os.path.join(pkg, "maps", "map.yaml")).read()
if "image: map.pgm" not in yaml:
    problems.append("map.yaml does not point at map.pgm")
res = float(re.search(r"resolution:\s*([\d.]+)", yaml).group(1))
ox, oy = (float(v) for v in re.search(r"origin:\s*\[([-\d.]+),\s*([-\d.]+)", yaml).groups())
x_min, y_min, x_max, y_max = make_map.DEFAULT_BOUNDS
if (ox, oy) != (x_min, y_min) or abs(width * res - (x_max - x_min)) > 1e-6 or abs(height * res - (y_max - y_min)) > 1e-6:
    problems.append("map.yaml origin/resolution do not match the %dx%d PGM and generator bounds" % (width, height))

with tempfile.TemporaryDirectory() as tmp:
    out = os.path.join(tmp, "map")
    grid = make_map.rasterize(make_map.load_shapes(os.path.join(pkg, "worlds", "sebastien.world")),
                              make_map.DEFAULT_BOUNDS, res, make_map.DEFAULT_SCAN_HEIGHT)
    make_map.write_pgm(out + ".pgm", grid)
    if open(out + ".pgm", "rb").read() != data:
        problems.append("maps/map.pgm is stale: rerun scripts/make_map.py worlds/sebastien.world maps/map")

world_launch = open(os.path.join(pkg, "launch", "world.launch")).read()
amcl_launch = open(os.path.join(pkg, "launch", "amcl.launch")).read()
def arg(text, name):
    return float(re.search(r'<arg name="%s" default="([-\d.]+)"' % name, text).group(1))
start = (arg(world_launch, "x"), arg(world_launch, "y"), arg(world_launch, "yaw"))
initial = (arg(amcl_launch, "initial_x"), arg(amcl_launch, "initial_y"), arg(amcl_launch, "initial_yaw"))
if start != initial:
    problems.append("Start Pose %s in world.launch differs from AMCL initial pose %s" % (start, initial))

for p in problems:
    print("  FAIL  " + p)
if problems:
    sys.exit(1)
print("  ok    map.pgm %dx%d @ %s m, in sync with the World; Start Pose = AMCL initial pose" % (width, height, res))
EOF

echo "Required files"
for f in config/costmap_common_params.yaml config/global_costmap_params.yaml config/local_costmap_params.yaml \
         config/base_local_planner_params.yaml rviz/localization.rviz; do
  if [ -f "$PKG/$f" ]; then ok "$f"; else bad "$f missing"; fi
done
if [ -f catkin_ws/src/teleop_twist_keyboard/teleop_twist_keyboard.py ]; then ok "teleop_twist_keyboard"; else bad "teleop_twist_keyboard missing"; fi

if command -v roslaunch >/dev/null 2>&1; then
  echo "roslaunch resolution (ROS found)"
  for l in world.launch amcl.launch; do
    if roslaunch --files my_robot "$l" >/dev/null 2>&1; then ok "$l"; else bad "$l (source devel/setup.bash first)"; fi
  done
else
  echo "roslaunch not found: skipping launch resolution (run this on the VM after catkin_make)"
fi

if [ "$fail" -ne 0 ]; then echo "Checks FAILED"; exit 1; fi
echo "All checks passed"
