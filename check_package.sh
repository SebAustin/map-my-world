#!/usr/bin/env bash
# Static checks that run without Gazebo: XML is well formed, RTAB-Map listens to the topics and frames the
# robot actually publishes, localization.launch differs from mapping.launch only as the brief says, and
# every poster Landmark has its texture. On the ROS VM it also asks roslaunch to resolve each launch file.
set -euo pipefail

cd "$(dirname "$0")"
PKG=catkin_ws/src/my_robot
fail=0
ok()  { echo "  ok    $1"; }
bad() { echo "  FAIL  $1"; fail=1; }

echo "XML well-formedness"
for f in "$PKG"/launch/*.launch "$PKG"/worlds/*.world "$PKG"/urdf/*.gazebo "$PKG"/urdf/*.xacro "$PKG"/package.xml \
         "$PKG"/models/landmarks/model.config "$PKG"/models/landmarks/model.sdf; do
  if xmllint --noout "$f" 2>/dev/null; then ok "$f"; else bad "$f"; fi
done

echo "Wiring and Landmark tests"
if ! python3 -c "import pytest" 2>/dev/null; then
  echo "  skip  pytest not installed (pip3 install pytest)"
elif python3 -m pytest -q "$PKG/test" >/dev/null 2>&1; then ok "pytest"
else bad "pytest (run: python3 -m pytest $PKG/test)"; fi

echo "Required files"
for f in launch/world.launch launch/mapping.launch launch/localization.launch launch/teleop.launch \
         rviz/mapping.rviz models/landmarks/materials/scripts/landmarks.material; do
  if [ -f "$PKG/$f" ]; then ok "$f"; else bad "$f missing"; fi
done
if [ -f catkin_ws/src/teleop_twist_keyboard/teleop_twist_keyboard.py ]; then ok "teleop_twist_keyboard"; else bad "teleop_twist_keyboard missing"; fi

if command -v roslaunch >/dev/null 2>&1; then
  echo "roslaunch resolution (ROS found)"
  for l in world.launch mapping.launch localization.launch teleop.launch; do
    if roslaunch --files my_robot "$l" >/dev/null 2>&1; then ok "$l"; else bad "$l (source devel/setup.bash first)"; fi
  done
  if rospack find rtabmap_ros >/dev/null 2>&1; then ok "rtabmap_ros installed"
  else bad "rtabmap_ros missing (sudo apt-get install ros-noetic-rtabmap-ros)"; fi
else
  echo "roslaunch not found: skipping launch resolution (run this on the VM after catkin_make)"
fi

if [ "$fail" -ne 0 ]; then echo "Checks FAILED"; exit 1; fi
echo "All checks passed"
