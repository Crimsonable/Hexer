cmake_minimum_required (VERSION 3.19)

target_sources(
    HexerCore 
    PRIVATE
        io.cpp
        mesh.cpp
        deformation.cpp
        transformation.cpp
        bfgs.cpp
        threadpool.cpp
        tessellation.cpp
    PUBLIC
        base.h
        traits.h
        expr.h
        io.h
        mesh.h
        deformation.h
        transformation.h
        bfgs.h
        solverbase.h
        utilities.h
        threadpool.h
        tessellation.h
        hexer_core.h
        expr_v2.h
)
target_include_directories(HexerCore PUBLIC ${CMAKE_CURRENT_LIST_DIR})