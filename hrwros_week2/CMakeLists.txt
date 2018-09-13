cmake_minimum_required(VERSION 2.8.3)

project(hrwros_week2)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY config launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES readme.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
