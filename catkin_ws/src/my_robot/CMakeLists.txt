cmake_minimum_required(VERSION 3.0.2)
project(my_robot)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY config launch maps meshes rviz urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/make_map.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
