cmake_minimum_required(VERSION 3.14)
project(CoordinateCreator)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_executable(CoordinateCreator
        agent.cpp
        agent.hpp
        main.cpp)
