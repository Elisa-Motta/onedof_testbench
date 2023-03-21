file(GLOB THIS_APP_SOURCES onedof_main.cpp onedof_app.cpp onedof_ease.cpp onedof_shared_memory_comm.cpp agree_joint_controller.cpp onedof_robot.cpp agree_common.cpp)
file(GLOB REHAMOVE_SOURCES onedof_rehamove/*.cpp)
file(GLOB REHAMOVE_HEADERS onedof_rehamove/*.h)
file(GLOB THIS_APP_HEADERS *.h)

get_filename_component(THIS_APP_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
message(STATUS "Example application, ${THIS_APP_NAME}, is ready to build")

add_executable(${THIS_APP_NAME} ${THIS_APP_SOURCES} ${THIS_APP_HEADERS})
target_link_libraries(${THIS_APP_NAME} pthread rt esmacat_core esmacat_driver)

file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/tmp/run.sh
"sudo ./${THIS_APP_NAME} &
sudo ${PROJECT_BINARY_DIR}/esmacat_core/application/ecat_main")

file (COPY ${CMAKE_CURRENT_BINARY_DIR}/tmp/run.sh DESTINATION ${CMAKE_CURRENT_BINARY_DIR} FILE_PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ)
file(REMOVE_RECURSE ${CMAKE_CURRENT_BINARY_DIR}/tmp)

###COPYING CONFIG FILES TO BINARY FOLDER
file(COPY configuration/onedof_config_files DESTINATION ${PROJECT_BINARY_DIR}/..)

add_executable(${THIS_APP_NAME}_input onedof_shared_memory_comm.cpp agree_input.cpp ${THIS_APP_HEADERS})
add_executable(${THIS_APP_NAME}_display onedof_shared_memory_comm.cpp agree_display.cpp ${THIS_APP_HEADERS})

### REHAMOVE

add_executable(${THIS_APP_NAME}_rehamove global_variables.h headers.h onedof_rehamove.cpp onedof_rehamove_app.cpp onedof_rehamove_class.cpp onedof_rehamove.h onedof_shared_memory_comm.cpp  ${REHAMOVE_SOURCES} ${REHAMOVE_HEADERS} ${THIS_APP_HEADERS})

include_directories(resource)
include_directories(onedof_rehamove)

add_library(libsmpt STATIC IMPORTED)
set_target_properties(libsmpt PROPERTIES
    IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/esmacat_applications/onedof_fesjoint_rebecca/onedof_rehamove/libsmpt.a)

target_link_libraries(${THIS_APP_NAME}_rehamove pthread rt libsmpt matio  libconfig++.a)


include_directories(configuration/onedof_config_files)

#add_subdirectory(onedof_rehamove)
