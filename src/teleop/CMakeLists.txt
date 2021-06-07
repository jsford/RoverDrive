cmake_minimum_required(VERSION 3.0.2)
project(teleop)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

catkin_package()

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pitranger
  geometry_msgs
)

## System dependencies are found with CMake's conventions
find_package(fmt REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare the teleop C++ Executable
add_executable(teleop_node
  src/teleop_node.cpp
  src/joystick/joystick.cc
)
add_dependencies(teleop_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(teleop_node
  ${catkin_LIBRARIES}
  fmt::fmt
)
