SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")

add_executable(colab_cloth colab_cloth.cpp)
target_link_libraries(colab_cloth custom_scene)

add_executable(rope_coverage rope_coverage.cpp)
target_link_libraries(rope_coverage custom_scene)
