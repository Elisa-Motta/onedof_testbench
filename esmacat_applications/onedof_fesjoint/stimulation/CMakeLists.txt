cmake_minimum_required(VERSION 2.8)

project(stimulation)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt5Core)


set(FES_MANAGER_SRC_FILES
        ${PROJECT_SOURCE_DIR}/src/feshandler.cpp
        ${PROJECT_SOURCE_DIR}/src/singlepulse.cpp)

set(FES_MANAGER_INC_FILES
        ${PROJECT_SOURCE_DIR}/inc/feshandler.h
        ${PROJECT_SOURCE_DIR}/inc/singlepulse.h)


add_library(fesManager STATIC ${FES_MANAGER_SRC_FILES})

target_link_libraries(fesManager Qt5::Core)

target_link_libraries(fesManager ncurses pthread)
