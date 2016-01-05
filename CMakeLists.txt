cmake_minimum_required(VERSION 2.6)
project( deform_simulator_old )

set( CATKIN_PACKAGES
    osg_bullet
    haptics
    bullet_helpers
)


## Find catkin macros and libraries
find_package( catkin REQUIRED COMPONENTS ${CATKIN_PACKAGES} )

## System dependencies are found with CMake's conventions
find_package( cmake_modules REQUIRED )
find_package( Boost REQUIRED COMPONENTS 
    system 
    filesystem 
    program_options
)

find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS    src
    LIBRARIES       
    CATKIN_DEPENDS  ${CATKIN_PACKAGES}
    DEPENDS         Eigen3
                    Boost
)

###########
## Build ##
###########

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -std=c++0x" )# -fopenmp" )

include_directories( src SYSTEM 
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
)

set( ${PROJECT_NAME}_LINK_TARGETS
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

add_subdirectory( src )
