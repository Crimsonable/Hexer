cmake_minimum_required(VERSION 3.19)

target_sources(
    SmoothEnergy
    PUBLIC
    test_utility.h
    SmoothEnergy.cpp
)

target_include_directories(SmoothEnergy PUBLIC ${CMAKE_CURRENT_LIST_DIR})