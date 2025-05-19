cmake_minimum_required(VERSION 3.15)
project(drone_navigation_ros VERSION 1.0.0)

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

# Download Eigen
if(NOT DEFINED EIGEN_LIB_ROOT_PATH)
  include(FetchContent)
  set(EIGEN_LIB_ROOT_PATH "${CMAKE_CURRENT_BINARY_DIR}/_deps/lib_eigen_src")
  message("Eigen library root path set to ${EIGEN_LIB_ROOT_PATH}")
  FetchContent_Declare(
    lib-eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
    SOURCE_DIR "${EIGEN_LIB_ROOT_PATH}")
  FetchContent_Populate(lib-eigen)
endif()

# # Download OpenCV library
# if(NOT DEFINED OPENCV_LIB_ROOT_PATH)
#   include(FetchContent)
#   set(OPENCV_LIB_ROOT_PATH "${CMAKE_CURRENT_BINARY_DIR}/_deps/lib_opencv_src")
#   message("OpenCV library root path set to ${OPENCV_LIB_ROOT_PATH}")
#   FetchContent_Declare(
#     lib-opencv
#     GIT_REPOSITORY https://github.com/opencv/opencv.git
#     GIT_TAG 4.11.0
#     SOURCE_DIR "${OPENCV_LIB_ROOT_PATH}")
#   FetchContent_MakeAvailable(lib-opencv)
# endif()

find_package(OpenCV REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(drone_navigation_msgs REQUIRED)

find_package(cv_bridge REQUIRED)

# ################################################################
# Add external projects
# ################################################################

# Add core algorithm
set(BUILD_FOR_ROS true)
set(DRONE_NAVIGATION_LIB_ROOT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/modules/drone_navigation/src")
add_subdirectory(modules/drone_navigation/src)

# ################################################################
# Build nodes
# ################################################################

# Add drone_vision_ros
set(TargetDroneVisionROS drone_vision_node)
add_executable(${TargetDroneVisionROS} 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/drone_vision/drone_vision_ros_exec.cpp 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/drone_vision/drone_vision_ros.cpp 
)
target_compile_features(${TargetDroneVisionROS} PUBLIC cxx_std_17)
ament_target_dependencies(${TargetDroneVisionROS} 
  PUBLIC 
    rclcpp 
    rclcpp_components 
    geometry_msgs 
    sensor_msgs 
    cv_bridge 
)
target_include_directories(${TargetDroneVisionROS} 
  PUBLIC 
    ${OpenCV_INCLUDE_DIRS} 
    ${cv_bridge_INCLUDE_DIRS} 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drone_vision 
)
target_link_libraries(${TargetDroneVisionROS} 
  PUBLIC 
    ${OpenCV_LIBS} 
    ${cv_bridge_LIBS} 
    drone_camera_wrapper 
    drone_yolo_wrapper 
)

# Add drone_planner_ros
set(TargetDronePlannerROS drone_planner_node)
add_executable(${TargetDronePlannerROS} 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/drone_planner/drone_planner_ros_exec.cpp 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/drone_planner/drone_planner_ros.cpp 
)
target_compile_features(${TargetDronePlannerROS} PUBLIC cxx_std_17)
ament_target_dependencies(${TargetDronePlannerROS} 
  PUBLIC 
    rclcpp 
    rclcpp_components 
    std_msgs 
    geometry_msgs 
    sensor_msgs 
    cv_bridge 
)
target_include_directories(${TargetDronePlannerROS} 
  PUBLIC 
    ${OpenCV_INCLUDE_DIRS} 
    ${EIGEN_LIB_ROOT_PATH} 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drone_planner 
)
target_link_libraries(${TargetDronePlannerROS} 
  PUBLIC 
    ${OpenCV_LIBS} 
    ${cv_bridge_LIBS} 
    drone_vfh_planner 
)

# Add flight_controller_ros
set(TargetFlightControllerROS drone_flight_controller_node)
add_executable(${TargetFlightControllerROS} 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/flight_control/flight_controller_ros_exec.cpp 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/flight_control/flight_controller_ros.cpp 
)
target_compile_features(${TargetFlightControllerROS} PUBLIC cxx_std_17)
ament_target_dependencies(${TargetFlightControllerROS} 
  PUBLIC 
    rclcpp 
    rclcpp_components 
    tf2 
    tf2_geometry_msgs 
    geometry_msgs 
    sensor_msgs 
    drone_navigation_msgs 
)
target_include_directories(${TargetFlightControllerROS} 
  PUBLIC 
    ${EIGEN_LIB_ROOT_PATH} 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/flight_control 
)
target_link_libraries(${TargetFlightControllerROS} 
  PUBLIC 
    drone_flight_controller 
    drone_state_estimator 
    drone_socket_udp 
)

# Add test class node
set(TargetDroneTestROS drone_test_node)
add_executable(${TargetDroneTestROS} 
  ${CMAKE_CURRENT_SOURCE_DIR}/test/drone_test_class_exec.cpp 
  ${CMAKE_CURRENT_SOURCE_DIR}/test/drone_test_class.cpp 
  ${CMAKE_CURRENT_SOURCE_DIR}/test/socketUDP.cpp 
)
target_compile_features(${TargetDroneTestROS} PUBLIC cxx_std_17)
ament_target_dependencies(${TargetDroneTestROS} 
  PUBLIC 
    rclcpp 
    rclcpp_components 
    std_msgs 
    geometry_msgs 
    drone_navigation_msgs 
)
target_include_directories(${TargetDroneTestROS} 
  PUBLIC 
    ${OpenCV_INCLUDE_DIRS} 
    ${EIGEN_LIB_ROOT_PATH}
)
target_link_libraries(${TargetDroneTestROS} 
  PUBLIC 
    ${OpenCV_LIBS}
)

# ################################################################
# Install files
# ################################################################

install(TARGETS 
  ${TargetDroneVisionROS} 
  ${TargetDronePlannerROS} 
  ${TargetFlightControllerROS} 
  ${TargetDroneTestROS} 
  DESTINATION lib/${PROJECT_NAME} 
)

install(
  DIRECTORY
    config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
