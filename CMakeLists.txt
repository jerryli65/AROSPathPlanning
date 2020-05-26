cmake_minimum_required(VERSION 3.16)
project(AROSPathPlanning)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    set(AROS_PATH_PLANNING_TESTING TRUE)
endif()

add_subdirectory(lib)
add_subdirectory(src)
