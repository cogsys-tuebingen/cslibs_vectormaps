cmake_minimum_required(VERSION 2.8.3)
project(utils_gdal)


find_package(catkin REQUIRED utils_boost_geometry)
find_package(GDAL   REQUIRED)
find_package(Boost  REQUIRED system filesystem)

find_package(OpenCV QUIET)
find_package(roscpp QUIET)
find_package(gazebo QUIET)
find_package(xacro  QUIET)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES utils_gdal
  CATKIN_DEPENDS utils_boost_geometry
  DEPENDS Boost
)

## Enforce that we use C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


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
add_executable(dxf_map_top_occupancy
    src/dxf_to_occupancy.cpp
)

target_link_libraries(dxf_map_top_occupancy
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${roscpp_LIBRARIES}
    utils_gdal
)

add_executable(dxf_map_renderer
    src/dxf_map_renderer.cpp
)

target_link_libraries(dxf_map_renderer
    ${OpenCV_LIBRARIES}
    utils_gdal
)

endif()

if(${gazebo_FOUND} AND ${xacro_FOUND})
    find_package(Eigen3 REQUIRED)
    include_directories(
        ${GAZEBO_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_DIRS}
    )

    add_library(gazebo_dxf_map_plugin
        src/dxf_map_gazebo.cpp
        src/mesh.cpp
    )
    target_link_libraries(gazebo_dxf_map_plugin
        ${GAZEBO_LIBRARIES}
        utils_gdal
    )
    xacro_add_xacro_file("${CMAKE_CURRENT_LIST_DIR}/gazebo/launch/empty.xacro"
                         "${CMAKE_CURRENT_LIST_DIR}/gazebo/launch/empty.world")

    add_custom_target(world ALL DEPENDS "gazebo/launch/empty.world")

    add_executable(dxf_to_world
        src/dxf_to_world.cpp
        src/mesh.cpp
    )
    target_link_libraries(dxf_to_world
        ${GAZEBO_LIBRARIES}
        utils_gdal
        yaml-cpp
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
