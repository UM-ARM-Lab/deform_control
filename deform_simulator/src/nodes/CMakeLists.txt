set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic" )

add_executable( custom_scene_node 
    custom_scene_node.cpp
)
add_dependencies( custom_scene_node
    custom_scene
    ${catkin_EXPORTED_DEPENENCIES}
)
target_link_libraries( custom_scene_node
    custom_scene
    ${catkin_LIBRARIES}
)
