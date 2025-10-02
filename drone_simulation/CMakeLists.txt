cmake_minimum_required(VERSION 3.15)
project(drone_simulation VERSION 1.0.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ################################################################
# Add external libraries
# ################################################################

find_package(OpenCV REQUIRED)

# Set gazebo variables
if("$ENV{GZ_VERSION}" STREQUAL "harmonic" OR NOT DEFINED "ENV{GZ_VERSION}")
  find_package(gz-cmake3 REQUIRED)
  set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

  gz_find_package(gz-common5 REQUIRED)
  set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

  gz_find_package(gz-rendering8 REQUIRED)
  set(GZ_RENDERING_VER ${gz-rendering8_VERSION_MAJOR})

  gz_find_package(gz-sim8 REQUIRED)
  set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

  message(STATUS "Compiling against Gazebo Harmonic")
else()  
  message(FATAL_ERROR "Unsupported GZ_VERSION: $ENV{GZ_VERSION}")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_sim REQUIRED)

find_package(drone_navigation_ros REQUIRED)

# ################################################################
# Add external projects
# ################################################################

# Add gazebo models (Ardupilot)
add_subdirectory(modules/ardupilot_gazebo)
# add_subdirectory(modules/gazebo_models)
# add_subdirectory(modules/gazebo_models_worlds_collection)

# ################################################################
# Build model plugins
# ################################################################

add_library(drone-rotor-control
  SHARED
    plugins/DroneRotorControl.cc
    plugins/DroneRotorControl.hh
    plugins/SocketUDP.cc
    plugins/SocketUDP.hh
)
target_link_libraries(drone-rotor-control PRIVATE
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)

add_library(drive-to-pose-controller
  SHARED
    plugins/DriveToPoseController.cc
    plugins/DriveToPoseController.hh
)
target_link_libraries(drive-to-pose-controller PRIVATE
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)

add_library(ackerman-controller
  SHARED
    plugins/AckermanController.cc
    plugins/AckermanController.hh
)
target_link_libraries(ackerman-controller PRIVATE
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)

# ################################################################
# Build ros nodes
# ################################################################

set(TargetPickupPoseCmd pickup_pose_cmd_node)
add_executable(${TargetPickupPoseCmd} 
  ${CMAKE_CURRENT_SOURCE_DIR}/nodes/vehicle_pose_cmd_ros_exec.cpp 
  ${CMAKE_CURRENT_SOURCE_DIR}/nodes/vehicle_pose_cmd_ros.cpp 
)
target_compile_features(${TargetPickupPoseCmd} PUBLIC cxx_std_17)
ament_target_dependencies(${TargetPickupPoseCmd} 
  PUBLIC 
    rclcpp 
    rclcpp_components 
    tf2 
    geometry_msgs 
)

# ################################################################
# Install files
# ################################################################

install(
  TARGETS
  drone-rotor-control
  drive-to-pose-controller
  ackerman-controller
  ${TargetPickupPoseCmd}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY
  models/
  DESTINATION share/${PROJECT_NAME}/models
)

install(
  DIRECTORY
  worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(
  DIRECTORY
  sim_files/
  DESTINATION share/${PROJECT_NAME}/sim_files
)

ament_package()
