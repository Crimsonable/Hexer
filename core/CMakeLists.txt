cmake_minimum_required (VERSION 3.19)

target_sources(
    Hexer 
    PRIVATE
        io.cpp
    PUBLIC
        base.h
        traits.h
        expr.h
        io.h
)
target_include_directories(Hexer PUBLIC ${CMAKE_CURRENT_LIST_DIR})