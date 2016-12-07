cmake_minimum_required(VERSION 2.8.3)
project(utils_gdal)


find_package(catkin REQUIRED utils_boost_geometry)
find_package(GDAL   REQUIRED)
find_package(Boost  REQUIRED system filesystem)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES utils_gdal
  CATKIN_DEPENDS utils_boost_geometry
  DEPENDS Boost
)

include(cmake/c++11.cmake)
include(cmake/optional_dependencies.cmake)
include(cmake/show_headers.cmake)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${GDAL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIR}
    ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS} ${Qt5OpenGL_INCLUDE_DIRS}
)

add_library(utils_gdal
   src/map/dxf_map.cpp
)

target_link_libraries(utils_gdal
    ${catkin_LIBRARIES}
    ${GDAL_LIBRARIES}
    ${Boost_LIBRARIES}
)

if(${OpenCV_FOUND} AND ${roscpp_FOUND})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${roscpp_INCLUDE_DIRS})
add_executable(gdal_dxf_map_to_occupancy
    src/tools/dxf_to_occupancy.cpp
)

target_link_libraries(gdal_dxf_map_to_occupancy
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${roscpp_LIBRARIES}
    utils_gdal
)

add_executable(gdal_dxf_map_renderer
    src/tools/dxf_map_renderer.cpp
)

target_link_libraries(gdal_dxf_map_renderer
    ${OpenCV_LIBRARIES}
    utils_gdal
)

endif()

if(${Qt5_FOUND} AND ${OPENGL_FOUND})

    add_definitions(${Qt5Widgets_DEFINITIONS})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
    set(CMAKE_AUTOMOC True)

    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
    include_directories(${CMAKE_CURRENT_BINARY_DIR})

    qt5_wrap_ui(QT_UI
                src/tools/map_viewer/map_viewer.ui
                src/tools/map_viewer/map_viewer_list_item.ui)

    add_executable(gdal_map_viewer
        src/tools/map_viewer/map_viewer.cpp
        src/tools/map_viewer/view.cpp
        src/tools/map_viewer/control.cpp
        src/tools/map_viewer/model.cpp

        src/tools/map_viewer/QInteractiveGraphicsView.hpp
        src/tools/map_viewer/QLayerListItem.hpp

        ${QT_UI}
    )

    target_link_libraries(gdal_map_viewer
        utils_gdal
        Qt5::Core Qt5::Gui Qt5::Widgets Qt5::OpenGL
        ${OPENGL_LIBRARIES}
    )



endif()
if(${gazebo_FOUND} AND ${xacro_FOUND} AND ${EIGEN3_FOUND})
    add_library(gazebo_dxf_map_plugin
        src/gazebo/dxf_map_gazebo.cpp
        src/gazebo/mesh.cpp
    )
    target_link_libraries(gazebo_dxf_map_plugin
        ${GAZEBO_LIBRARIES}
        utils_gdal
    )
    xacro_add_xacro_file("${CMAKE_CURRENT_LIST_DIR}/gazebo/launch/empty.xacro"
                         "${CMAKE_CURRENT_LIST_DIR}/gazebo/launch/empty.world")

    add_custom_target(world ALL DEPENDS "gazebo/launch/empty.world")

    add_executable(dxf_to_world
        src/gazebo/dxf_to_world.cpp
        src/gazebo/mesh.cpp
    )
    target_link_libraries(dxf_to_world
        ${GAZEBO_LIBRARIES}
        utils_gdal
        yaml-cpp
    )

endif()
