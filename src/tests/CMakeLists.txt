#add_executable(test_simplescene test_simplescene.cpp)
#target_link_libraries(test_simplescene simulation robots)

#add_executable(test_softbody test_softbody.cpp)
#target_link_libraries(test_softbody simulation)

#add_executable(test_cloth_grasping test_cloth_grasping.cpp)
#target_link_libraries(test_cloth_grasping simulation robots)

#add_executable(test_rigidbody_grasping test_rigidbody_grasping.cpp)
#target_link_libraries(test_rigidbody_grasping simulation robots)

#add_executable(test_playback_joints test_playback_joints.cpp)
#target_link_libraries(test_playback_joints simulation)

#add_executable(test_rope test_rope.cpp)
#target_link_libraries(test_rope simulation robots)

#add_executable(test_rope_fork test_rope_fork.cpp)
#target_link_libraries(test_rope_fork simulation robots)

#add_executable(test_fork test_fork.cpp)
#target_link_libraries(test_fork simulation)

#add_executable(test_fork_softbody test_fork_softbody.cpp)
#target_link_libraries(test_fork_softbody simulation)

#add_executable(test_shirt test_shirt.cpp)
#target_link_libraries(test_shirt simulation)

#add_executable(test_collision_detection test_collision_detection.cpp)
#target_link_libraries(test_collision_detection simulation)

#add_executable(test_action test_action.cpp)
#target_link_libraries(test_action simulation)

#add_executable(test_scene_record test_scene_record.cpp)
#target_link_libraries(test_scene_record simulation)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDEFORM_SIMULATOR_OLD -Wall -Wno-deprecated-declarations -Wno-reorder -std=c++0x" )


# Disable the gcc warning about no unknown pragmas when we have -fopenmp disabled
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unknown-pragmas")
endif()


add_executable(test_colab_cloth
    test_colab_cloth.cpp
    colab_cloth.h
    colab_cloth_defines.h
    gripper_kinematic_object.cpp
    gripper_kinematic_object.h
    manual_gripper_path.cpp
    manual_gripper_path.h
    internal_utils.hpp
    )
target_link_libraries(test_colab_cloth simulation_old)
