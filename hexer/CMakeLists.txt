cmake_minimum_required(VERSION 3.19)

target_sources(
    Hexer
    PUBLIC
        Hexer.cpp
        Hexer.h   
)

target_include_directories(Hexer PUBLIC ${CMAKE_CURRENT_LIST_DIR})