set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic" )

add_executable( cloth_colab_folding cloth_colab_folding.cpp )
target_link_libraries(cloth_colab_folding custom_scene)

add_executable( cloth_coverage cloth_coverage.cpp )
target_link_libraries( cloth_coverage custom_scene )

add_executable( rope_coverage rope_coverage.cpp )
target_link_libraries( rope_coverage custom_scene )
