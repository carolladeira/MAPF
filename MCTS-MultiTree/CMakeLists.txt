cmake_minimum_required(VERSION 3.14)
project(MCTS_MultiTree)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_executable(MCTS_MultiTree
        agent.cpp
        agent.hpp
        main.cpp
        mcts.cpp
        mcts.hpp
        ScenarioLoader.cpp
        ScenarioLoader.h
        sim.cpp
        sim.hpp
        tree.cpp
        tree.hpp)
