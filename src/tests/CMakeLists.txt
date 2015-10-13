add_executable(test_colab_cloth
    custom_key_handler.cpp
    test_colab_cloth.cpp
    colab_cloth_custom_scene.cpp
    gripper_kinematic_object.cpp
)
target_link_libraries(test_colab_cloth simulation)
