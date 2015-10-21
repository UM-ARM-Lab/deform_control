cmake_minimum_required( VERSION 2.8.3 )
project( osg_old )

find_package( catkin REQUIRED )

file(GLOB OSG_LIBRARIES "lib64/*.so")

catkin_package(
    INCLUDE_DIRS    include
    LIBRARIES       ${OSG_LIBRARIES}
    CATKIN_DEPENDS
    DEPENDS         system_lib
)
