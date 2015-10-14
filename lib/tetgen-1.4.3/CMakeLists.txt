set_source_files_properties(predicates.cxx PROPERTIES COMPILE_FLAGS "-O0")
set_source_files_properties(tetgen.cxx PROPERTIES COMPILE_FLAGS "-O2")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-int-to-pointer-cast -Wno-unused-result")
add_library(tetgen STATIC tetgen.cxx predicates.cxx)
