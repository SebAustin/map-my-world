cmake_minimum_required(VERSION 3.0.2)
project(my_robot)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch meshes models rviz urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/make_landmarks.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
