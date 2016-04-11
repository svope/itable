cmake_minimum_required(VERSION 2.8.3)
project(itable_demo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(catkin REQUIRED)
SET(CMAKE_CXX_FLAGS "-std=c++0x")
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}" ${CMAKE_MODULE_PATH})
message (${CMAKE_SOURCE_DIR})
find_package(SFML 2 REQUIRED system window graphics network audio)
find_package(OpenCV REQUIRED )
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  pcl_conversions
  pcl_ros
  cv_bridge
)

 
 catkin_package(
  INCLUDE_DIRS include
  LIBRARIES itable_pkg
  CATKIN_DEPENDS message_runtime roscpp std_msgs
  DEPENDS system_lib
)

 
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS} ${SFML_INCLUDE_DIR}
)

## Declare a C++ executable
add_executable(itable_demo_node src/itable_demo_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(itable_demo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
 					itable_pkg_)

## Specify libraries to link a library or executable target against
 target_link_libraries(itable_demo_node
  ${OpenCV_LIBS} ${catkin_LIBRARIES} ${SFML_LIBRARIES} -lsfeMovie
 )


