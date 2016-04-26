cmake_minimum_required(VERSION 2.8.3)
project(itable_calib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  image_geometry
  cv_bridge
  OpenCV
)

include(FindPkgConfig)
pkg_search_module(SDL2 REQUIRED sdl2)
PKG_SEARCH_MODULE(SDL2IMAGE REQUIRED SDL2_image)
include_directories(${SDL2_INCLUDE_DIRS} ${SDL2IMAGE_INCLUDE_DIRS})

#find_package(
#  PCL 1.2 REQUIRED
#)


#find_package(Boost REQUIRED COMPONENTS system)

set(CMAKE_BUILD_TYPE Debug)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES itable_calib
  CATKIN_DEPENDS roscpp tf #image_transport
  DEPENDS system_lib
)

set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_C_COMPILER "clang")

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a cpp executable
add_executable(itable_calib src/itable_calib.cpp
                                        )

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(kinect2_superpixels_node kinect2_superpixels_generate_messages_cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(itable_calib
   ${catkin_LIBRARIES} ${SDL2_LIBRARIES} ${SDL2IMAGE_LIBRARIES}
)
