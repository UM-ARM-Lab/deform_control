SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic" )

add_library(custom_scene
    custom_scene.cpp
    custom_key_handler.cpp
    gripper_kinematic_object.cpp
)
target_link_libraries( custom_scene 
    simulation
)
