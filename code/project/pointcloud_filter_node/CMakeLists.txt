cmake_minimum_required(VERSION 3.1.10)
project(pointcloud_filter_node)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=deprecated-declarations -Wno-deprecated-declarations -Wno-deprecated -Wno-cpp")
set(CMAKE_BUILD_TYPE "Release")

find_package(PCL REQUIRED QUIET)
message(STATUS "PCL_VERSION is ${PCL_VERSION}")
include_directories(${PCL_INCLUDE_DIRS})

list(FILTER catkin_LIBRARIES EXCLUDE REGEX /usr/lib/x86_64-linux-gnu/libpcl*)
list(FILTER catkin_LIBRARIES EXCLUDE REGEX optimized)
list(FILTER catkin_LIBRARIES EXCLUDE REGEX debug)
list(FILTER catkin_INCLUDE_DIRS EXCLUDE REGEX /usr/include/pcl-1.10)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pointcloud_filter_node src/pointcloud_filter_node.cpp)
target_link_libraries(pointcloud_filter_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})