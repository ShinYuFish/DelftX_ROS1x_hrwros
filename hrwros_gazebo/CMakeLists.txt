cmake_minimum_required(VERSION 2.8.3)
project(hrwros_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

#if(NOT WIN32)
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
#endif()

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  gazebo_msgs
  gazebo_plugins
  gazebo_ros
  geometry_msgs
  message_generation
  roscpp
  std_msgs
  std_srvs
  tf
)

find_package(gazebo REQUIRED)

find_package(TinyXML REQUIRED)

########################
## Message generation ##
########################

add_message_files(
  DIRECTORY
    msg
  FILES
    ConveyorBeltState.msg
    VacuumGripperState.msg
    Proximity.msg
    Model.msg
    LogicalCameraImage.msg
)

add_service_files(
  DIRECTORY
    srv
  FILES
    ConveyorBeltControl.srv
    VacuumGripperControl.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    conveyor_spawner
    urdf_creator
  CATKIN_DEPENDS
    gazebo_msgs
    gazebo_plugins
    gazebo_ros
    geometry_msgs
    message_runtime
    roscpp
    std_msgs
    std_srvs
    tf
  DEPENDS
    TinyXML
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

###########
## Build ##
###########

# Create URDF creator library
add_library(urdf_creator
  src/urdf_creator.cpp
)
target_link_libraries(urdf_creator
  ${catkin_LIBRARIES}
  ${TinyXML_LIBRARIES}
)

# Create conveyor spawner library
add_library(conveyor_spawner
  src/conveyor_spawner.cpp
)
target_link_libraries(conveyor_spawner
  ${catkin_LIBRARIES}
  urdf_creator
)

# Create conveyor spawner executable
add_executable(conveyor_spawner_node
  src/conveyor_spawner_node.cpp
)
target_link_libraries(conveyor_spawner_node
  ${catkin_LIBRARIES}
  conveyor_spawner
)

# Create URDF creator test node
add_executable(urdf_creator_test_node
  src/urdf_creator_test.cpp
)
target_link_libraries(urdf_creator_test_node
  ${catkin_LIBRARIES}
  urdf_creator
)

# Create the libSideContactPlugin.so library.
set(side_contact_plugin_name SideContactPlugin)
add_library(${side_contact_plugin_name} src/plugins/SideContactPlugin.cc)
target_link_libraries(${side_contact_plugin_name}
  ${GAZEBO_LIBRARIES}
)
#install(TARGETS ${side_contact_plugin_name}
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin
#)

# Create the libVacuumGripperPlugin.so library.
set(vacuum_gripper_plugin_name VacuumGripperPlugin)
add_library(${vacuum_gripper_plugin_name} src/plugins/VacuumGripperPlugin.cc)
target_link_libraries(${vacuum_gripper_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
)
#install(TARGETS ${vacuum_gripper_plugin_name}
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin)

# Create the libROSVacuumGripperPlugin.so library.
set(ros_vacuum_gripper_plugin_name ROSVacuumGripperPlugin)
add_library(${ros_vacuum_gripper_plugin_name} src/plugins/ROSVacuumGripperPlugin.cc)
target_link_libraries(${ros_vacuum_gripper_plugin_name}
  ${catkin_LIBRARIES}
  ${vacuum_gripper_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
)
add_dependencies(${ros_vacuum_gripper_plugin_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
#install(TARGETS ${ros_vacuum_gripper_plugin_name}
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin)

# Create the libProximityRayPlugin.so library.
set(proximity_ray_plugin_name ProximityRayPlugin)
add_library(${proximity_ray_plugin_name} src/plugins/ProximityRayPlugin.cc)
target_link_libraries(${proximity_ray_plugin_name}
  ${GAZEBO_LIBRARIES}
)
install(TARGETS ${proximity_ray_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create the libROSProximityRayPlugin.so library.
set(ros_proximity_ray_plugin_name ROSProximityRayPlugin)
add_library(${ros_proximity_ray_plugin_name} src/plugins/ROSProximityRayPlugin.cc)
target_link_libraries(${ros_proximity_ray_plugin_name}
  ${proximity_ray_plugin_name}
  ${GAZEBO_LIBRARIES}
)
add_dependencies(${ros_proximity_ray_plugin_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
install(TARGETS ${ros_proximity_ray_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create the libConveyorBeltPlugin.so library.
set(conveyor_belt_plugin_name ConveyorBeltPlugin)
add_library(${conveyor_belt_plugin_name} src/plugins/ConveyorBeltPlugin.cc)
target_link_libraries(${conveyor_belt_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${side_contact_plugin_name}
)
#install(TARGETS ${conveyor_belt_plugin_name}
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin
#)

# Create the libROSConveyorBeltPlugin.so library.
set(ros_conveyor_belt_plugin_name ROSConveyorBeltPlugin)
add_library(${ros_conveyor_belt_plugin_name} src/plugins/ROSConveyorBeltPlugin.cc)
target_link_libraries(${ros_conveyor_belt_plugin_name}
  ${conveyor_belt_plugin_name}
  ${GAZEBO_LIBRARIES}
)
add_dependencies(${ros_conveyor_belt_plugin_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
#install(TARGETS ${ros_conveyor_belt_plugin_name}
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin
#)

# Create the libObjectDisposalPlugin.so library.
set(object_disposal_plugin_name ObjectDisposalPlugin)
add_library(${object_disposal_plugin_name} src/plugins/ObjectDisposalPlugin.cc)
target_link_libraries(${object_disposal_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${side_contact_plugin_name}
)
#install(TARGETS ${object_disposal_plugin_name}
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin
#)

# Create the libROSLogicalCameraPlugin.so library.
set(ros_logical_camera_plugin_name ROSLogicalCameraPlugin)
add_library(${ros_logical_camera_plugin_name} src/plugins/ROSLogicalCameraPlugin.cc)
target_link_libraries(${ros_logical_camera_plugin_name}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
)
add_dependencies(${ros_logical_camera_plugin_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
install(TARGETS ${ros_logical_camera_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
