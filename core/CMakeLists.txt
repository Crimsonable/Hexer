cmake_minimum_required (VERSION 3.19)

target_sources(
    Hexer 
    PRIVATE
        graph.cpp
        meshIO.cpp
    PUBLIC
        base.h
        graph.h
        ops.h
        op.h
        traits.h
        expr.h
)
target_include_directories(Hexer PUBLIC ${CMAKE_CURRENT_LIST_DIR})