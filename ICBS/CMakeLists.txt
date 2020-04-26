cmake_minimum_required(VERSION 3.14)
project(ICBS)

set(CMAKE_CXX_STANDARD 14)

add_executable(ICBS main.cpp Agent.cpp Endpoint.cpp compute_heuristic.cpp egraph_reader.cpp ICBSNode.cpp ICBSSearch.cpp LLNode.cpp map_loader.cpp MDD.cpp node.cpp Simulation.cpp SingleAgentICBS.cpp SingleSearch.cpp ScenarioLoader.cpp ScenarioLoader.h)