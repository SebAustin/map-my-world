# Map My World

The differential-drive robot from Where Am I? drives around its building under keyboard control and builds its own picture of it, in 2D and 3D, using RTAB-Map with an RGB-D camera and its lidar.

## Language

**World**:
The simulated building the robot lives in: two rooms, their walls, the Furniture and the Landmarks.
_Avoid_: environment, scene, map

**Furniture**:
A full-height obstacle placed in the World so that no two places in the building look alike to the lidar.
_Avoid_: objects, models

**Landmark**:
Something in the World with enough visual texture for the camera to recognise it again when the robot comes back past it. A textured piece of Furniture is also a Landmark: the two words name roles, not separate objects.
_Avoid_: feature, marker

**Map**:
The 2D occupancy grid the robot builds while it drives, saying which cells are walls or Furniture and which are free.
_Avoid_: grid map, world, costmap

**3D Map**:
The coloured 3D point cloud of the World the robot builds alongside the Map.
_Avoid_: octomap, cloud, point cloud

**Mapping Run**:
One session from starting the mapper to shutting it down, producing exactly one Database.
_Avoid_: session, mapping session

**Database**:
The file a Mapping Run leaves behind, holding every camera image, the robot's path, the Map and the 3D Map.
_Avoid_: db, rtabmap.db

**Loop Closure**:
The moment the robot recognises a place it has already been from what the camera sees, and corrects its path to match.
_Avoid_: relocalization, match

**Start Pose**:
Where the robot is placed in the World at launch: the middle doorway, facing east.
_Avoid_: spawn point, initial position
