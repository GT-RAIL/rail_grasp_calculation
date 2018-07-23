cmake_minimum_required(VERSION 2.8.3)
project(rail_grasp_calculation_nodes)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  cmake_modules
  eigen_conversions
  geometry_msgs
  interactive_markers
  pcl_conversions
  pcl_ros
  rail_grasp_calculation_msgs
  rail_manipulation_msgs
  roscpp
  std_srvs
  tf_conversions
  visualization_msgs
)

find_package(Eigen REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(grasp_sampler
  src/GraspSampler.cpp src/PoseWithHeuristic.cpp
)
add_dependencies(grasp_sampler
  rail_manipulation_msgs_gencpp
)
target_link_libraries(grasp_sampler
  ${catkin_LIBRARIES}
  ${EIGEN_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS grasp_sampler
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
