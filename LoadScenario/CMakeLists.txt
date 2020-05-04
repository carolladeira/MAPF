cmake_minimum_required(VERSION 3.14)
project(LoadScenario)

set(CMAKE_CXX_STANDARD 14)

add_executable(LoadScenario main.cpp ScenarioLoader.cpp MapLoader.cpp MapLoader.h)