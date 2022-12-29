cmake_minimum_required(VERSION 3.19)

target_sources(
    Smoother
    PUBLIC
        Smoother.cpp
)

target_include_directories(Smoother PUBLIC ${CMAKE_CURRENT_LIST_DIR})