cmake_minimum_required(VERSION 3.16)
project(AROSPathPlanning)

set(CMAKE_CXX_STANDARD 17)

add_library(AROSPathPlanning library.cpp library.hpp)
