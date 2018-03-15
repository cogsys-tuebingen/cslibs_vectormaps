cmake_minimum_required(VERSION 3.1)
project(cslibs_vectormaps)

find_package(catkin REQUIRED cslibs_boost_geometry cslibs_indexed_storage)
find_package(GDAL   REQUIRED)

# Optional Dependencies
find_package(gazebo QUIET)
find_package(xacro  QUIET)
find_package(Qt5 COMPONENTS Core Gui Widgets QUIET)
find_package(Boost REQUIRED COMPONENTS system iostreams regex)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

catkin_package(
  INCLUDE_DIRS     include
  LIBRARIES        ${PROJECT_NAME}_dxf ${PROJECT_NAME}_maps
  CATKIN_DEPENDS   cslibs_boost_geometry
  DEPENDS          Boost GDAL
)

# C++ 11
set(CMAKE_CXX_STANDARD 11)
# OpenMP
set(OpenMP true)
include(cmake/cslibs_vectormaps_omp.cmake)
# Show the headers
include(cmake/cslibs_vectormaps_show_headers.cmake)

add_library(${PROJECT_NAME}_dxf
   src/dxf/dxf_map.cpp
)
target_include_directories(${PROJECT_NAME}_dxf
    PUBLIC
        include
        ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_dxf
    ${catkin_LIBRARIES}
    ${GDAL_LIBRARIES}
    ${Boost_LIBRARIES}
)

add_library(${PROJECT_NAME}_maps
    src/maps/vector_map.cpp
    src/maps/grid_vector_map.cpp
    src/maps/simple_grid_vector_map.cpp
    src/maps/oriented_grid_vector_map.cpp
    src/maps/oriented_visibility_grid_vector_map.cpp
    src/maps/rtree_vector_map.cpp
)
target_include_directories(${PROJECT_NAME}_maps
    PUBLIC
        include
        ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_maps
    ${catkin_LIBRARIES}
    ${GDAL_LIBRARIES}
    ${Boost_LIBRARIES}
    yaml-cpp
)

add_executable(${PROJECT_NAME}_load_map
    src/test/load_map.cpp
)
target_include_directories(${PROJECT_NAME}_load_map
    PRIVATE include
)
target_link_libraries(${PROJECT_NAME}_load_map
    ${catkin_LIBRARIES}
    ${GDAL_LIBRARIES}
    ${Boost_LIBRARIES}
    ${PROJECT_NAME}_maps
)


if(${Qt5_FOUND})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
    set(CMAKE_AUTOMOC True)

    qt5_wrap_ui(QT_UI
        src/map_viewer/ui/map_viewer.ui
        src/map_viewer/ui/map_viewer_list_item.ui
        src/map_viewer/ui/map_viewer_corner_param_dialog.ui
        src/map_viewer/ui/map_viewer_gridmap_param_dialog.ui
        src/map_viewer/ui/map_viewer_vectormap_param_dialog.ui
        src/map_viewer/ui/map_viewer_rtree_vectormap_param_dialog.ui
    )

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
        src/map_viewer/models/polygon_layer_model.cpp

        src/map_viewer/qt/QInteractiveGraphicsView.hpp
        src/map_viewer/qt/QLayerListItem.hpp
        src/map_viewer/qt/QCornerParamDialog.hpp
        src/map_viewer/qt/QGridmapParamDialog.hpp
        src/map_viewer/qt/QVectormapParamDialog.hpp
        src/map_viewer/qt/QRtreeVectormapParamDialog.hpp

        src/map_viewer/util/rng_color.hpp

        src/map_viewer/algorithms/corner_detection.cpp
        src/map_viewer/algorithms/rasterization.cpp
        src/map_viewer/algorithms/vectormap_conversion.cpp
        src/map_viewer/algorithms/rtree_vectormap_conversion.cpp

        ${QT_UI}
    )
    target_include_directories(${PROJECT_NAME}_map_viewer
        PRIVATE
            include
            ${CMAKE_CURRENT_SOURCE_DIR}/src
            ${CMAKE_CURRENT_BINARY_DIR}
    )
    target_link_libraries(${PROJECT_NAME}_map_viewer
        ${PROJECT_NAME}_dxf
        ${PROJECT_NAME}_maps
        Qt5::Core Qt5::Gui Qt5::Widgets Threads::Threads
        ${catkin_LIBRARIES}
    )

endif()

find_package(protobuf QUIET)
if(${protobuf_FOUND} AND ${gazebo_FOUND} AND ${xacro_FOUND})
    if(${protobuf_VERSION} VERSION_EQUAL 2.6.1)
        find_package(Eigen3 REQUIRED)
        find_package(Boost  REQUIRED system filesystem)

        add_library(${PROJECT_NAME}_gazebo_dxf_map_plugin
            src/gazebo/dxf_map_gazebo.cpp
            src/gazebo/mesh.cpp
        )
        target_include_directories(${PROJECT_NAME}_gazebo_dxf_map_plugin
            PUBLIC
                include
                ${Eigen3_INCLUDE_DIR}
                ${Boost_INCLUDE_DIRS}
        )
        target_link_libraries(${PROJECT_NAME}_gazebo_dxf_map_plugin
            ${GAZEBO_LIBRARIES}
            ${PROJECT_NAME}_dxf
        )

        xacro_add_xacro_file("${CMAKE_CURRENT_LIST_DIR}/gazebo/launch/empty.xacro"
                             "${CMAKE_CURRENT_LIST_DIR}/gazebo/launch/empty.world"
                             INORDER)

        add_custom_target(${PROJECT_NAME}_world ALL DEPENDS "gazebo/launch/empty.world")

        add_executable(${PROJECT_NAME}_dxf_to_gazebo_world
            src/gazebo/dxf_to_world.cpp
            src/gazebo/mesh.cpp
        )
        target_include_directories(${PROJECT_NAME}_gazebo_dxf_map_plugin
            PUBLIC
                include
                ${Eigen3_INCLUDE_DIR}
                ${Boost_INCLUDE_DIRS}
        )
        target_link_libraries(${PROJECT_NAME}_dxf_to_gazebo_world
            ${GAZEBO_LIBRARIES}
            ${Boost_LIBRARIES}
            ${PROJECT_NAME}_dxf
            yaml-cpp
        )
    endif()
endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

