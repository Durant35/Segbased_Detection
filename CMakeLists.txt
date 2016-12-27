cmake_minimum_required(VERSION 2.8.3)
project(test_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  tf
  tf_conversions
  libpointmatcher_ros
)

find_package(libpointmatcher REQUIRED)

add_definitions(-std=c++11 -O2 -g -DNDEBUG)

catkin_package (
#	CATKIN_DEPENDS roscpp
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${libpointmatcher_INCLUDE_DIRS}
)

add_executable(test_detection src/test_detection.cpp)
target_link_libraries(test_detection 
${catkin_LIBRARIES}
${libpointmatcher_LIBRARIES}
)
