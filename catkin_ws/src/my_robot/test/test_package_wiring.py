"""Static checks that the robot, the World and the RTAB-Map launch files agree with each other.
These catch at the desk what would otherwise only show up as a silent RTAB-Map on the VM.
Run with: python3 -m pytest catkin_ws/src/my_robot/test"""

import os
import re
import xml.etree.ElementTree as ET

import pytest

PKG = os.path.join(os.path.dirname(__file__), "..")
LAUNCH = os.path.join(PKG, "launch")
SENSORS = ("rgb/image", "depth/image", "rgb/camera_info", "scan")


def read(*parts):
    with open(os.path.join(PKG, *parts)) as f:
        return f.read()


def parse(*parts):
    return ET.parse(os.path.join(PKG, *parts)).getroot()


def launch_args(root):
    return {a.get("name"): a.get("default") for a in root.findall("arg")}


def resolve(value, args):
    return re.sub(r"\$\(arg (\w+)\)", lambda m: args[m.group(1)], value)


def rtabmap_node(root):
    return next(n for n in root.iter("node") if n.get("type") == "rtabmap")


def node_params(node):
    return {p.get("name"): p.get("value") for p in node.findall("param")}


def node_remaps(node, args):
    return {r.get("from"): resolve(r.get("to"), args) for r in node.findall("remap")}


def published_topics():
    """Absolute topics the Gazebo plugins publish, keyed by what they carry."""
    gazebo = parse("urdf", "my_robot.gazebo")
    kinect = next(p for p in gazebo.iter("plugin") if p.get("filename") == "libgazebo_ros_openni_kinect.so")
    ns = "/" + kinect.findtext("cameraName")
    laser = next(p for p in gazebo.iter("plugin") if p.get("filename") == "libgazebo_ros_laser.so")
    return {
        "rgb/image": ns + "/" + kinect.findtext("imageTopicName"),
        "depth/image": ns + "/" + kinect.findtext("depthImageTopicName"),
        "rgb/camera_info": ns + "/" + kinect.findtext("cameraInfoTopicName"),
        "scan": laser.findtext("topicName"),
    }


@pytest.fixture(params=["mapping.launch", "localization.launch"])
def rtabmap_launch(request):
    root = parse("launch", request.param)
    return root, rtabmap_node(root), launch_args(root)


def test_every_launch_file_is_well_formed():
    for name in os.listdir(LAUNCH):
        if name.endswith(".launch"):
            parse("launch", name)


def test_rtabmap_listens_to_the_topics_the_robot_publishes(rtabmap_launch):
    _, node, args = rtabmap_launch
    remaps = node_remaps(node, args)
    published = published_topics()
    for sensor in SENSORS:
        assert remaps[sensor] == published[sensor], sensor


def test_rtabmap_frame_is_the_robot_base_link(rtabmap_launch):
    _, node, _ = rtabmap_launch
    xacro = read("urdf", "my_robot.xacro")
    base = re.search(r'<joint name="robot_footprint_joint".*?<parent link="(\w+)"/>', xacro, re.S).group(1)
    assert node_params(node)["frame_id"] == base


def test_rtabmap_odom_frame_is_what_the_diff_drive_publishes(rtabmap_launch):
    _, node, _ = rtabmap_launch
    gazebo = parse("urdf", "my_robot.gazebo")
    drive = next(p for p in gazebo.iter("plugin") if p.get("filename") == "libgazebo_ros_diff_drive.so")
    assert node_params(node)["odom_frame_id"] == drive.findtext("odometryFrame")


def test_rtabmapviz_matches_rtabmap_inputs():
    root = parse("launch", "mapping.launch")
    args = launch_args(root)
    viz = next(n for n in root.iter("node") if n.get("type") == "rtabmapviz")
    assert node_remaps(viz, args) == {k: v for k, v in node_remaps(rtabmap_node(root), args).items() if k in SENSORS}
    assert node_params(viz)["frame_id"] == node_params(rtabmap_node(root))["frame_id"]


def test_mapping_starts_a_fresh_database_and_skips_idle_data():
    node = rtabmap_node(parse("launch", "mapping.launch"))
    assert "--delete_db_on_start" in (node.get("args") or "")
    assert node_params(node)["Mem/NotLinkedNodesKept"] == "false"
    assert "Mem/IncrementalMemory" not in node_params(node)


def test_localization_differs_from_mapping_only_as_the_brief_says():
    mapping_root, local_root = parse("launch", "mapping.launch"), parse("launch", "localization.launch")
    mapping, local = rtabmap_node(mapping_root), rtabmap_node(local_root)
    assert launch_args(local_root) == launch_args(mapping_root)
    assert "--delete_db_on_start" not in (local.get("args") or "")
    expected = dict(node_params(mapping))
    del expected["Mem/NotLinkedNodesKept"]
    expected["Mem/IncrementalMemory"] = "false"
    assert node_params(local) == expected
    args = launch_args(mapping_root)
    assert node_remaps(local, args) == node_remaps(mapping, args)


def test_camera_optical_frame_hangs_off_the_camera_link():
    xacro = read("urdf", "my_robot.xacro")
    joint = re.search(r'<joint name="camera_optical_joint".*?</joint>', xacro, re.S).group(0)
    assert '<parent link="camera_link"/>' in joint
    assert '<child link="camera_link_optical"/>' in joint
    assert '<link name="camera_link_optical">' in xacro
    kinect = next(p for p in parse("urdf", "my_robot.gazebo").iter("plugin")
                  if p.get("filename") == "libgazebo_ros_openni_kinect.so")
    assert kinect.findtext("frameName") == "camera_link_optical"


def test_every_poster_in_the_world_has_a_material_and_texture():
    world = read("worlds", "sebastien.world")
    material = read("models", "landmarks", "materials", "scripts", "landmarks.material")
    textures = os.path.join(PKG, "models", "landmarks", "materials", "textures")
    names = re.findall(r"<name>(Landmark/Poster\w)</name>", world)
    assert len(names) == len(set(names)) >= 3
    for name in names:
        block = re.search(r"material %s\s*\{.*?texture (\S+)" % re.escape(name), material, re.S)
        assert block, name
        assert os.path.isfile(os.path.join(textures, block.group(1))), block.group(1)


def test_posters_are_visual_only_so_the_map_shows_bare_walls():
    world = parse("worlds", "sebastien.world")
    posters = next(m for m in world.iter("model") if m.get("name") == "posters")
    assert posters.findall(".//collision") == []


def test_teleop_launch_starts_the_vendored_keyboard_node():
    node = parse("launch", "teleop.launch").find("node")
    assert (node.get("pkg"), node.get("type")) == ("teleop_twist_keyboard", "teleop_twist_keyboard.py")
    assert {p.get("name") for p in node.findall("param")} >= {"key_timeout", "speed", "turn"}


def test_gazebo_can_find_the_landmark_model():
    assert 'gazebo_model_path="${prefix}/models"' in read("package.xml")
