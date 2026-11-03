# Where Am I?

Udacity Robotics Software Engineer, project 3. The differential-drive robot from [Go Chase It](https://github.com/SebAustin/go-chase-it) finds and keeps track of its Pose Estimate on a known Map of its building, using AMCL on lidar and odometry. It can be driven by keyboard or sent Navigation Goals through `move_base`. Vocabulary is in [GLOSSARY.md](GLOSSARY.md).

```
catkin_ws/src/
├── my_robot/
│   ├── config/        move_base costmap and local planner parameters
│   ├── launch/        world.launch (Gazebo + RViz), amcl.launch (map_server, amcl, move_base)
│   ├── maps/          map.pgm + map.yaml, generated from the World
│   ├── meshes/        hokuyo.dae
│   ├── rviz/          localization.rviz
│   ├── scripts/       make_map.py (World -> Map)
│   ├── test/          pytest for make_map.py
│   ├── urdf/          my_robot.xacro, my_robot.gazebo
│   └── worlds/        sebastien.world: two rooms plus Furniture
└── teleop_twist_keyboard/   vendored from ros-teleop
docs/screenshots/      submission screenshots
check_package.sh       static checks (no Gazebo needed)
```

## The World and the Map

The building is two 4 × 6 m rooms joined by a 1.8 m doorway, with an opening in the east wall. On its own, that layout is mirror-symmetric: a robot in the west room facing east sees nearly the same scan as one in the east room facing west, so AMCL can split into two clusters. Six full-height pieces of Furniture break the symmetry:

| Room | Furniture | Position |
|---|---|---|
| West | bookshelf 1.6 × 0.4 m | north wall, (−2.5, 2.7) |
| West | pillar, r 0.25 m | (−1.5, −1.5) |
| West | L-shaped counter | SW corner |
| East | crate 0.8 × 0.8 m | (2.0, 1.6) |
| East | cabinet 0.5 × 1.0 m | (3.5, −2.2) |

The Map is generated from the World file by `scripts/make_map.py` instead of `pgm_map_creator`, which was written for Kinetic and needs CMake patches on Noetic. The script reads every box and cylinder collision that crosses the lidar's scan height (0.27 m), so the Map lines up with the Gazebo walls exactly. Output: 200 × 160 cells at 0.05 m, origin (−5, −4), covering the building with 1 m of margin.

![Map](docs/map.png)

Regenerate after editing the World:

```bash
cd catkin_ws/src/my_robot
python3 scripts/make_map.py worlds/sebastien.world maps/map
```

## Robot changes from Go Chase It

- Lidar widened from 180° to 270° (±2.356 rad, like a Hokuyo UTM-30LX), 720 samples on `/scan`. More walls per scan means faster convergence.
- Lidar raised 5 cm so its beams (0.27 m) clear the wheel tops (0.24 m); otherwise the rear of the wider scan would see the robot's own wheels.
- Start Pose moved to the middle doorway, (0, 0) facing east, which is also the AMCL initial pose.

## Setup (Udacity VM, ROS Noetic)

```bash
sudo apt-get update
sudo apt-get install -y ros-noetic-navigation ros-noetic-map-server ros-noetic-move-base ros-noetic-amcl
git clone https://github.com/SebAustin/where-am-i.git ~/where-am-i
cd ~/where-am-i/catkin_ws && catkin_make && source devel/setup.bash
```

## Run

```bash
roslaunch my_robot world.launch    # terminal 1: Gazebo + RViz
roslaunch my_robot amcl.launch     # terminal 2: map_server, amcl, move_base
```

RViz opens in the `map` frame. It shows the Map, both costmaps, the lidar scan (red), the global plan (green) and the Particle Cloud (blue arrows). The map-dependent displays turn green once `amcl.launch` is up.

Drive the robot in either of two ways:

- **Navigation Goal:** click **2D Nav Goal** in RViz and drag on the Map. `move_base` plans and drives there.
- **Teleop:** in a third terminal, `rosrun teleop_twist_keyboard teleop_twist_keyboard.py` (`i` forward, `j`/`l` turn, `k` stop).

To test recovery, use **2D Pose Estimate** to put the Pose Estimate somewhere wrong, then drive. The cloud spreads out and then re-converges on the true pose.

## AMCL parameters (`launch/amcl.launch`)

| Parameter | Value | Why |
|---|---|---|
| `min_particles` / `max_particles` | 500 / 2500 | Enough to cover the start covariance; AMCL shrinks toward 500 once converged |
| `initial_pose_x/y/a` | 0, 0, 0 | The Start Pose |
| `initial_cov_xx/yy/aa` | 0.25, 0.25, 0.07 | 0.5 m and ~15° of starting doubt |
| `update_min_d` / `update_min_a` | 0.1 m / 0.2 rad | Frequent updates at the robot's ≤ 0.5 m/s |
| `transform_tolerance` | 0.5 | Avoids TF timeout warnings on the VM's slow sim clock |
| `recovery_alpha_slow/fast` | 0.001 / 0.1 | Injects random particles when scans stop matching, so a kidnapped robot recovers |
| `laser_model_type` | likelihood_field | Cheap and robust for walls and Furniture |
| `laser_max_beams` | 60 | Every 12th of the 720 beams, plenty for a small building |
| `laser_z_hit` / `laser_z_rand` | 0.95 / 0.05 | The simulated lidar is clean (σ 1 cm) |
| `laser_likelihood_max_dist` | 2.0 m | Distance over which the likelihood field is computed |
| `odom_model_type` | diff-corrected | Differential drive |
| `odom_alpha1..4` | 0.005 | Gazebo odometry is ground truth; diff-corrected expects small values (the 0.2 default is far too noisy) |

## move_base parameters (`config/`)

Based on Udacity's `config.zip`, retuned for this robot:

| Parameter | Value | Why |
|---|---|---|
| `footprint` | 0.5 × 0.3 m rectangle | The real chassis instead of a 0.3 m circle |
| `inflation_radius` | 0.35 m | Keeps clear of walls; the 1.8 m doorway stays passable |
| `obstacle_range` / `raytrace_range` | 2.5 / 3.0 m | Marks nearby obstacles and clears free space behind them |
| `observation_sources` topic | `/scan` | This robot's lidar topic |
| costmap update / publish | 5 / 2 Hz | Avoids "map update loop missed its desired rate" warnings |
| local costmap | 4 × 4 m rolling | Small and fast, built from live lidar only |
| `max_vel_x` / `max_vel_theta` | 0.5 m/s / 1.0 rad/s | Slow enough for AMCL to keep up |

## Stopping for obstacles

The local costmap is built only from the live lidar, so anything that appears in the robot's path gets marked within one scan. Try it: send a Navigation Goal across the building, then in Gazebo drop a box (Insert → Box) in front of the robot. The box shows up in the local costmap, and the robot stops and replans around it. If no path exists, it runs `move_base`'s recovery behaviours (clear costmaps, rotate in place) instead of driving into it.

## Screenshots

| | |
|---|---|
| Just launched: Particle Cloud spread around the Start Pose | ![start](docs/screenshots/01_start.png) |
| After a Navigation Goal into the east room: converged | ![east](docs/screenshots/02_east_room.png) |
| After teleop back into the west room: still converged | ![west](docs/screenshots/03_west_room.png) |
| Kidnapped with a wrong 2D Pose Estimate, then recovered | ![recovered](docs/screenshots/04_recovered.png) |

## Checks

```bash
./check_package.sh
```

This checks that the XML is well formed, runs the map-generator tests, confirms that `map.pgm` is in sync with the World and matches `map.yaml`, and confirms that the Start Pose equals the AMCL initial pose. On the VM, after `catkin_make`, it also resolves both launch files with `roslaunch --files`.
