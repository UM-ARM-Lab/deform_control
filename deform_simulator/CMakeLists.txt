cmake_minimum_required(VERSION 3.1.0)
project(deform_simulator)

# http://doc.qt.io/qt-5/cmake-manual.html
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)
# Create code from a list of Qt designer ui files
set(CMAKE_AUTOUIC ON)

# Find the QtWidgets library
find_package(Qt5Widgets CONFIG REQUIRED)
set(QT5_LIBS Qt5::Widgets)


set(CATKIN_PACKAGES
    actionlib
    arc_utilities
    geometry_msgs
    visualization_msgs
    roscpp
    osg_bullet
    haptics
    bullet_helpers
    sdf_tools
    sensor_msgs
    tf2_ros
    tf2_geometry_msgs
    deformable_manipulation_msgs
    deformable_manipulation_experiment_params
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGES})

## System dependencies are found with CMake's conventions
find_package(cmake_modules REQUIRED)
find_package(Boost REQUIRED COMPONENTS
    system
    filesystem
    program_options
    thread
)

find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

################################################
## Declare ROS messages, services and actions ##
################################################

# Declared in the smmap_msgs package

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS    include
#    LIBRARIES       ${PROJECT_NAME}
    CATKIN_DEPENDS  ${CATKIN_PACKAGES}
    DEPENDS         Eigen3
                    Boost
)


###########
## Build ##
###########

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -g -O0 -Werror=return-type -fsanitize=address -fno-omit-frame-pointer")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -g -O2 -Werror=return-type")

include_directories(include SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
)

set(${PROJECT_NAME}_LINK_TARGETS
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

add_subdirectory(src)
add_subdirectory(nodes)
