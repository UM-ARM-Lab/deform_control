set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations" )

add_library(simulation_old
    environment.cpp 
    basicobjects.cpp 
    #openravesupport.cpp 
    util.cpp 
    simplescene.cpp 
    softbodies.cpp 
    rope.cpp 
    plotting.cpp 
    convexdecomp.cpp 
    recording.cpp 
    config_bullet.cpp 
    config_viewer.cpp 
    bullet_io.cpp
)
add_dependencies( simulation_old
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries( simulation_old
    ${${PROJECT_NAME}_LINK_TARGETS}
    utils_old
)

