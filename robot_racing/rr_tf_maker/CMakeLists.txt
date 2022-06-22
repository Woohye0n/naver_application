cmake_minimum_required(VERSION 3.0.2)
project(rr_tf_maker)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rr_common
  roscpp
  tf
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES rr_tf_maker
  CATKIN_DEPENDS roscpp tf rr_common
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rr_tf_maker 
  src/rr_tf_maker/main.cpp
  src/rr_tf_maker/tf_maker.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies( rr_tf_maker rr_tf_maker)

## Specify libraries to link a library or executable target against
target_link_libraries( rr_tf_maker ${catkin_LIBRARIES}
)
