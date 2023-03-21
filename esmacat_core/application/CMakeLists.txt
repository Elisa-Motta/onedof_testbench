add_executable(ecat_main ecat_main.cpp)
target_link_libraries(ecat_main pthread rt esmacat_core ethercat_driver)
set_target_properties(ecat_main PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")

if(ROS_ENABLE)
    add_custom_command(TARGET ecat_main POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:ecat_main> ${CMAKE_BINARY_DIR}/devel/lib/${PROJECT_NAME})
endif()

add_executable(slaveinfo ../../SOEM/test/linux/slaveinfo/slaveinfo.c)
target_link_libraries(slaveinfo ethercat_driver)
set_target_properties(slaveinfo PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")

if(ROS_ENABLE)
    add_custom_command(TARGET slaveinfo POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:slaveinfo> ${CMAKE_BINARY_DIR}/devel/lib/${PROJECT_NAME})
endif()
