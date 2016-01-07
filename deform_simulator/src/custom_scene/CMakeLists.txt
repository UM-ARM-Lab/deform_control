SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -Wextra -Wpedantic -Wconversion" )

add_library( custom_scene
    ../../include/custom_scene/custom_scene.h
    custom_scene.cpp
    ../../include/custom_scene/gripper_kinematic_object.h
    gripper_kinematic_object.cpp
    ../../include/custom_scene/manual_gripper_path.h
    manual_gripper_path.cpp
)
add_dependencies ( custom_scene
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencpp
)
target_link_libraries( custom_scene
    simulation
    ${${PROJECT_NAME}_LINK_TARGETS}
)
