include_directories( SYSTEM
    ${BULLETSIM_SOURCE_DIR}/lib/haptics
)

add_library(utils
    config.cpp 
    vector_io.cpp 
    yes_or_no.cpp
    util.cpp
)

target_link_libraries(utils
    haptics
)
