cmake_minimum_required(VERSION 3.0.2)
project(rm_windmill_speed)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS"-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(catkin REQUIRED
        roscpp
        tf
        rm_common
        message_generation
        dynamic_reconfigure
        std_msgs
        geometry_msgs
        visualization_msgs
        image_transport
        cv_bridge
        image_geometry
        nodelet
        tf2
        tf2_geometry_msgs
        rm_msgs
        )

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

generate_dynamic_reconfigure_options(
        cfg/Windmill.cfg
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        rm_common
        roscpp
        tf
        message_generation
        dynamic_reconfigure
        std_msgs
        geometry_msgs
        visualization_msgs
        image_transport
        cv_bridge
        image_geometry
        nodelet
        tf2
        tf2_geometry_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

FILE(GLOB_RECURSE SOURCE_FILES "src/*.cpp")

add_library(
        ${PROJECT_NAME}
        src/windmill_speed.cpp
        )

target_link_libraries(
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${CERES_LIBRARIES}
)