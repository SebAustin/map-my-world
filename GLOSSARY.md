# Where Am I?

A simulated differential-drive robot that works out where it is inside a known map of its building, using its lidar and odometry.

## Language

**World**:
The simulated building the robot lives in: two rooms, their walls, and the Furniture.
_Avoid_: environment, scene, map

**Furniture**:
A full-height obstacle placed in the World so that no two places in the building look alike to the lidar.
_Avoid_: objects, models, landmarks

**Map**:
The occupancy picture of the World the robot is given in advance, saying which cells are walls or Furniture and which are free.
_Avoid_: world, costmap

**Start Pose**:
Where the robot is placed in the World at launch: the middle doorway, facing east.
_Avoid_: spawn point, initial position

**Pose Estimate**:
The robot's best belief of where it is on the Map and which way it faces.
_Avoid_: position, location, localization result

**Particle Cloud**:
The set of candidate poses the robot is weighing at once; tightly bunched means the robot is confident of its Pose Estimate.
_Avoid_: pose array, particles, arrows

**Navigation Goal**:
A pose on the Map the robot is asked to drive itself to.
_Avoid_: target, nav goal, waypoint
