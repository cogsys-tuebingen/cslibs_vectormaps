cmake_minimum_required(VERSION 2.8.3)
project(cslibs_vectormaps)

find_package(catkin REQUIRED cslibs_boost_geometry cslibs_indexed_storage)
find_package(GDAL   REQUIRED)

# Optional Dependencies
find_package(gazebo QUIET)
find_package(xacro  QUIET)
find_package(Qt5 COMPONENTS Core Gui Widgets QUIET)

catkin_package(
  INCLUDE_DIRS     include
  LIBRARIES        ${PROJECT_NAME}
  CATKIN_DEPENDS   cslibs_boost_geometry
  DEPENDS          Boost GDAL
)

# C++ 11
include(cmake/cslibs_vectormaps_c++11.cmake)
# OpenMP
set(OpenMP true)
include(cmake/cslibs_vectormaps_omp.cmake)
# Show the headers
include(cmake/cslibs_vectormaps_show_headers.cmake)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${GDAL_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
   src/dxf/dxf_map.cpp
   src/vector/vector_map_grid.cpp
   src/vector/vector_map_grid_orientation.cpp
   src/vector/vector_map_grid_visibility.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${GDAL_LIBRARIES}
    ${Boost_LIBRARIES}
)

if(${Qt5_FOUND})
    set(CMAKE_EXE_LINKER_FLAGS "-lpthread ${CMAKE_EXE_LINKER_FLAGS}")
    add_definitions(-lpthread ${Qt5Widgets_DEFINITIONS})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
    set(CMAKE_AUTOMOC True)

    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src
                        ${CMAKE_CURRENT_BINARY_DIR}
                        ${OpenCV_INCLUDE_DIRS}
    )

    qt5_wrap_ui(QT_UI
                src/map_viewer/ui/map_viewer.ui
                src/map_viewer/ui/map_viewer_list_item.ui
                src/map_viewer/ui/map_viewer_corner_param_dialog.ui
                src/map_viewer/ui/map_viewer_gridmap_param_dialog.ui)

    add_executable(${PROJECT_NAME}_map_viewer
        src/map_viewer/map_viewer.cpp
        src/map_viewer/view.cpp
        src/map_viewer/control.cpp
        src/map_viewer/map.cpp
        src/map_viewer/renderer.cpp
        src/map_viewer/parameters.cpp

        src/map_viewer/models/layer_model.cpp
        src/map_viewer/models/point_layer_model.cpp
        src/map_viewer/models/corner_layer_model.cpp
        src/map_viewer/models/vector_layer_model.cpp
        src/map_viewer/models/door_layer_model.cpp

        src/map_viewer/qt/QInteractiveGraphicsView.hpp
        src/map_viewer/qt/QLayerListItem.hpp
        src/map_viewer/qt/QCornerParamDialog.hpp
        src/map_viewer/qt/QGridmapParamDialog.hpp

        src/map_viewer/util/rng_color.hpp

        src/map_viewer/algorithms/corner_detection.cpp
        src/map_viewer/algorithms/rasterization.cpp

        ${QT_UI}

    )

    target_link_libraries(${PROJECT_NAME}_map_viewer
        ${PROJECT_NAME}
        Qt5::Core Qt5::Gui Qt5::Widgets
        yaml-cpp
        ${catkin_LIBRARIES}
    )

endif()
if(${gazebo_FOUND} AND ${xacro_FOUND})
    find_package(Eigen3 REQUIRED)
    find_package(Boost  REQUIRED system filesystem)

    include_directories(
        ${Eigen3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
    )

    add_library(${PROJECT_NAME}_gazebo_dxf_map_plugin
        src/gazebo/dxf_map_gazebo.cpp
        src/gazebo/mesh.cpp
    )
    target_link_libraries(${PROJECT_NAME}_gazebo_dxf_map_plugin
        ${GAZEBO_LIBRARIES}
        ${PROJECT_NAME}
    )

    xacro_add_xacro_file("${CMAKE_CURRENT_LIST_DIR}/gazebo/launch/empty.xacro"
                         "${CMAKE_CURRENT_LIST_DIR}/gazebo/launch/empty.world"
                         INORDER)

    add_custom_target(${PROJECT_NAME}_world ALL DEPENDS "gazebo/launch/empty.world")

    add_executable(${PROJECT_NAME}_dxf_to_gazebo_world
        src/gazebo/dxf_to_world.cpp
        src/gazebo/mesh.cpp
    )
    target_link_libraries(${PROJECT_NAME}_dxf_to_gazebo_world
        ${GAZEBO_LIBRARIES}
        ${Boost_LIBRARIES}
        ${PROJECT_NAME}
        yaml-cpp
    )
endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

