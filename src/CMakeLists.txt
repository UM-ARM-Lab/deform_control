add_definitions("-DBULLETSIM_DATA_DIR=${BULLETSIM_SOURCE_DIR}/data")
include_directories( SYSTEM
    ${BULLETSIM_SOURCE_DIR}/src
    ${BULLET_DIR}/src
    ${OSG_INCLUDE_DIRS}
    ${OSGBULLET_DIR}
    ${OSGWORKS_DIR}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${JSON_INCLUDE_DIR}
)
link_directories(${BULLETSIM_BINARY_DIR}/src)

add_subdirectory(simulation)
add_subdirectory(utils)
add_subdirectory(tests)
