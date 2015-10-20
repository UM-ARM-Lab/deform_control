cmake_minimum_required( VERSION 2.8.3 )
project( osg )

find_package( catkin REQUIRED )

find_library( OSG_LIBRARY
    NAMES osg
    PATHS lib64 
    NO_DEFAULT_PATH
)

#set(OSG_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/lib64/libosg.so")

catkin_package(
    INCLUDE_DIRS    include
    LIBRARIES       ${OSG_LIBRARY}
)
