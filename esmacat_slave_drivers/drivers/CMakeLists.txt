file(GLOB JC_SOURCES
    actuator_controller_interface.cpp
    joint_structs.cpp
    joint_controller.cpp
    length_acquisition.cpp
    load_acquisition.cpp
    position_acquisition.cpp)

add_library(joint_controller ${JC_SOURCES})
target_link_libraries(joint_controller soem esmacat_core)
