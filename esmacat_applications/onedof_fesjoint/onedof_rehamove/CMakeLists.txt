cmake_minimum_required(VERSION 2.8)
file(GLOB THIS_APP_SOURCES *.cpp)
file(GLOB THIS_APP_HEADERS *.h)

#project(realtime_rehamove)
include_directories({CMAKE_SOURCE_DIR}/esmacat_applications/onedof_esmacat_rt)


get_filename_component(THIS_APP_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
message(STATUS "Example application, ${THIS_APP_NAME}, is ready to build")

include_directories(resource)
add_executable(${THIS_APP_NAME} ${THIS_APP_SOURCES} ${THIS_APP_HEADERS})

add_library(libsmpt STATIC IMPORTED)
set_target_properties(libsmpt PROPERTIES
    IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/esmacat_applications/rehamove/libsmpt.a)

target_link_libraries(${THIS_APP_NAME} pthread rt libsmpt)

#target_link_libraries(main PRIVATE Qt5::Widgets libsmpt
 #   Qt5::Core)
