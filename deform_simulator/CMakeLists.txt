cmake_minimum_required( VERSION 2.8.3 )
project( deform_simulator )

set( CATKIN_PACKAGES
    arc_utilities
    geometry_msgs
    roscpp
    osg_bullet
    haptics
)

## Find catkin macros and libraries
find_package( catkin REQUIRED COMPONENTS ${CATKIN_PACKAGES} message_generation )

# external libraries
find_package( cmake_modules REQUIRED )
find_package( Boost REQUIRED COMPONENTS 
    system 
    filesystem 
    program_options
    thread
)

find_package( Eigen3 REQUIRED )

#find_package( OpenThreads REQUIRED )
#find_package( osg REQUIRED )
#find_package( osgDB REQUIRED )
#find_package( osgGA REQUIRED )
#find_package( osgText REQUIRED )
#find_package( osgUtil REQUIRED )
#find_package( osgViewer REQUIRED )

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    GripperTrajectory.msg 
    GripperTrajectoryStamped.msg
    SimulatorFbkStamped.msg
)

## Generate services in the 'srv' folder
add_service_files(
    FILES
    GetGripperNames.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#     FILES
#     Action1.action
#     Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    geometry_msgs
    std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS    include
#    LIBRARIES       ${PROJECT_NAME}
    CATKIN_DEPENDS  ${CATKIN_PACKAGES}
                    message_runtime
    DEPENDS         Eigen3
                    Boost
)


###########
## Build ##
###########

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fopenmp -std=c++0x" )
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -fopenmp -std=c1x" )

include_directories( include SYSTEM 
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
)

set( ${PROJECT_NAME}_LINK_TARGETS
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

add_subdirectory( src )
add_subdirectory( nodes )
