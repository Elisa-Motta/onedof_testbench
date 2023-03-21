file(GLOB THIS_APP_SOURCES onedof_main.cpp onedof_app.cpp shared_memory_comm.cpp)

get_filename_component(THIS_APP_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
message(STATUS "Example application, ${THIS_APP_NAME}, is ready to build")

add_executable(${THIS_APP_NAME} ${THIS_APP_SOURCES} )
target_link_libraries(${THIS_APP_NAME} pthread rt esmacat_core esmacat_driver)

file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/tmp/run.sh
"sudo ./${THIS_APP_NAME} &
sudo ${PROJECT_BINARY_DIR}/esmacat_core/application/ecat_main")

file (COPY ${CMAKE_CURRENT_BINARY_DIR}/tmp/run.sh DESTINATION ${CMAKE_CURRENT_BINARY_DIR} FILE_PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ)
file(REMOVE_RECURSE ${CMAKE_CURRENT_BINARY_DIR}/tmp)

###COPYING CONFIG FILES TO BINARY FOLDER
file(COPY configuration/onedof_config_files DESTINATION ${PROJECT_BINARY_DIR}/..)

add_executable(test_input shared_memory_comm.cpp user_input.cpp)
add_executable(test_display shared_memory_comm.cpp display.cpp)


