cmake_minimum_required(VERSION 2.8.3)
project(BezierTrajGenerator)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        nav_core
        roscpp
        std_msgs
        nav_msgs
        visualization_msgs
        geometry_msgs
        )

find_package(Eigen3 REQUIRED)
#find_package(serial)
#find_package(Ceres REQUIRED)
find_package(OsqpEigen)

catkin_package(
        INCLUDE_DIRS include
        #  LIBRARIES trajectory_planning
        CATKIN_DEPENDS nav_core roscpp std_msgs
        #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
)

add_library(MapManager
        include/MapManager/MapManager.h
        src/MapManager/MapManager.cpp)

add_library(AstarPathPlanner
        include/AstarPathPlanner/AstarPathPlanner.h
        src/AstarPathPlanner/AstarPathPlanner.cpp)
target_link_libraries(AstarPathPlanner
        ${catkin_LIBRARIES}
        MapManager)

add_library(BezierTrajGenerator
        include/BezierTrajGenerator/BezierTrajGenerator.h
        src/BezierTrajGenerator/BezierTrajGenerator.cpp)
target_link_libraries(BezierTrajGenerator
        ${catkin_LIBRARIES}
        ${CERES_LIBRARIES}
        OsqpEigen::OsqpEigen)

add_executable(BezierNode App/Bezier_node.cpp
        )
target_link_libraries(BezierNode
        ${catkin_LIBRARIES}
        AstarPathPlanner
        BezierTrajGenerator
        )

