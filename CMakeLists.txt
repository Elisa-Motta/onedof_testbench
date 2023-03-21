project(esmacat_rt LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

cmake_minimum_required(VERSION 3.10)

message("Starting Esmacat cmake")
message("Finding system configuration")
message("-- CMAKE_SYSTEM_NAME:      ${CMAKE_SYSTEM_NAME}")
message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
message("-- CMAKE_SYSTEM:           ${CMAKE_SYSTEM}")

string(FIND ${CMAKE_SYSTEM} "esmacat-master-s" esmacat_master_hw_type_s)
if ( NOT (${esmacat_master_hw_type_s} STREQUAL "-1"))
    message ("Esmacat hardware is Esmacat Master S")
    include_directories(configuration/hw/esmacat_master_s/)
endif()
string(FIND ${CMAKE_SYSTEM} "rockchip" esmacat_master_hw_type_c)
if ( NOT (${esmacat_master_hw_type_c} STREQUAL "-1"))
    message ("Esmacat hardware is Esmacat Master C with TinkerBoard")
    include_directories(configuration/include/hw/esmacat_master_c/)
endif()

string(FIND ${CMAKE_SYSTEM} "4.19.71-rt24-v7l+" esmacat_master_hw_type_c)
if ( NOT (${esmacat_master_hw_type_c} STREQUAL "-1"))
    message ("Esmacat hardware is Esmacat Master C with Rpi4")
    include_directories(configuration/include/hw/esmacat_master_c/)
endif()

include_directories(esmacat_core/include)
include_directories(esmacat_slave_drivers/drivers)
include_directories(resource)
include_directories(configuration)
include_directories(SOEM/soem)
include_directories(SOEM/osal)
include_directories(SOEM/osal/linux)
include_directories(SOEM/oshw/linux)
include_directories(resource/eigen)
include_directories(resource/file_handling/include/json/single_include/nlohmann)
include_directories(resource/file_handling/include)

message(STATUS "esmacat_core library creating")
file(GLOB ESMACAT_CORE_SOURCES esmacat_core/src/*.cpp)

add_library(esmacat_core STATIC
    ${ESMACAT_CORE_SOURCES}
    )
target_link_libraries(esmacat_core pthread rt)

message(STATUS "file handling library creating")
file(GLOB FILE_HANDLING_SOURCES resource/file_handling/src/*.cpp)
add_library(file_handling STATIC
    ${FILE_HANDLING_SOURCES}
    )

message(STATUS "ethercat_driver library creating")
file(GLOB SOEM_SOURCES SOEM/soem/*.c)
file(GLOB OSAL_SOURCES SOEM/osal/linux/*.c)
file(GLOB OSHW_SOURCES SOEM/oshw/linux/*.c)
add_library(ethercat_driver STATIC
    ${SOEM_SOURCES}
    ${OSAL_SOURCES}
    ${OSHW_SOURCES}
    )
target_link_libraries(ethercat_driver pthread)

message(STATUS "esmacat_driver library creating")
file(GLOB ESMACAT_DRIVER_SOURCES esmacat_slave_drivers/drivers/*.cpp)
add_library(esmacat_driver STATIC
    ${ESMACAT_DRIVER_SOURCES}
    )
target_link_libraries(esmacat_driver file_handling)

link_directories(${PROJECT_BINARY_DIR}/lib)

install(TARGETS esmacat_core DESTINATION ${PROJECT_BINARY_DIR}/lib)
install(TARGETS esmacat_driver DESTINATION ${PROJECT_BINARY_DIR}/lib)
install(TARGETS ethercat_driver DESTINATION ${PROJECT_BINARY_DIR}/lib)

add_subdirectory(esmacat_applications)
add_subdirectory(esmacat_core/application)
add_subdirectory(esmacat_slave_drivers)

message("-- PROJECT_DIR: ${PROJECT_BINARY_DIR}")
