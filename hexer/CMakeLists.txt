cmake_minimum_required(VERSION 3.19)

target_sources(
    Hexer
    PUBLIC
        Hexer.cpp
)

target_include_directories(Hexer PUBLIC ${CMAKE_CURRENT_LIST_DIR})