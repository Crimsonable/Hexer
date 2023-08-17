cmake_minimum_required(VERSION 3.19)

target_sources(
    Viewer
    PUBLIC
        viewer.cpp
)

target_include_directories(Viewer PUBLIC ${CMAKE_CURRENT_LIST_DIR})