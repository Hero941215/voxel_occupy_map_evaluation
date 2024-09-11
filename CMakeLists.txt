cmake_minimum_required(VERSION 3.0.2)
project(voxel_occupy)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-std=c++11 -DEIGEN_DONT_PARALLELIZE")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  message_generation
  message_runtime
  nav_msgs
  pcl_conversions
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  rosbag
  image_transport
  visualization_msgs
)

# third library
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV 3.4.1 REQUIRED QUIET)
# find_package(GTSAM REQUIRED QUIET)
find_package(Ceres REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# ignore warning
add_definitions(-w)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL
  CATKIN_DEPENDS cv_bridge geometry_msgs message_generation message_runtime nav_msgs pcl_conversions roscpp rospy sensor_msgs std_msgs tf visualization_msgs
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}  
  ${OpenCV_INCLUDE_DIRS}
)

# third library path
link_directories(
  include
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${CERES_LIBRARY_DIRS}
)

# VoxelOccupy 用于重建地图质量评估
add_executable(VoxelOccupy src/VoxelOccupy.cpp)
target_link_libraries(VoxelOccupy ${catkin_LIBRARIES} ${PCL_LIBRARIES})



