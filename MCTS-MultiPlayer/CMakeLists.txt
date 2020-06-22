cmake_minimum_required(VERSION 3.14)
project(MCTS-MultiPlayer)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_executable(mcts
        agent.cpp
        agent.hpp
        main.cpp
        mcts.cpp
        mcts.hpp
        sim.cpp
        sim.hpp
        tree.cpp
        tree.hpp
        ScenarioLoader.h
        ScenarioLoader.cpp)
