cmake_minimum_required (VERSION 3.19)

target_sources(
    HexerCore 
    PRIVATE
        io.cpp
        filter.cpp
        mesh.cpp
        kd_tree.cpp
        deformation.cpp
        transformation.cpp
        bfgs.cpp
    PUBLIC
        base.h
        traits.h
        expr.h
        io.h
        filter.h
        mesh.h
        kd_tree.h
        deformation.h
        transformation.h
        bfgs.h
        solverbase.h
        utilities.h
)
target_include_directories(HexerCore PUBLIC ${CMAKE_CURRENT_LIST_DIR})