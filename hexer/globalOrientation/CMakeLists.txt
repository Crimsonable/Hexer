cmake_minimum_required(VERSION 3.19)

target_sources(
    GlobalOrientation
    PUBLIC
        GlobalOrientation.cpp
)

target_include_directories(GlobalOrientation PUBLIC ${CMAKE_CURRENT_LIST_DIR})