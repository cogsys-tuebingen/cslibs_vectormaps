cmake_minimum_required(VERSION 2.8.3)
project(utils_gdal)


find_package(catkin REQUIRED utils_boost_geometry)
find_package(GDAL   REQUIRED)
find_package(Boost  REQUIRED)

find_package(OpenCV QUIET)
find_package(roscpp QUIET)


set(CMAKE_BUILD_TYPE Release)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES utils_gdal
  CATKIN_DEPENDS utils_boost_geometry
  DEPENDS Boost
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${GDAL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_library(utils_gdal
   src/dxf_map.cpp
 )

target_link_libraries(utils_gdal
    ${catkin_LIBRARIES}
    ${GDAL_LIBRARIES}
    ${Boost_LIBRARIES}
)

if(${OpenCV_FOUND} AND ${roscpp_FOUND})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${roscpp_INCLUDE_DIRS})
add_executable(dxf_map_visualizer
    src/dxf_map_visualizer.cpp
)

target_link_libraries(dxf_map_visualizer
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${roscpp_LIBRARIES}
    utils_gdal
)
endif()

## Declare a cpp executable
# add_executable(utils_gdal_node src/utils_gdal_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(utils_gdal_node utils_gdal_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(utils_gdal_node
#   ${catkin_LIBRARIES}
# )
