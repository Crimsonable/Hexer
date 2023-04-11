cmake_minimum_required (VERSION 3.19)

target_sources(
    HexerCore
    PRIVATE
        non.cpp 
    PUBLIC
        base.h
        traits.h
        expr.h
        mesh.h
        deformation.h
        transformation.h
        bfgs.h
        solverbase.h
        utilities.h
        threadpool.h
        tessellation.h
        hexer_core.h
        cycle.h
        old_deformation.h
        cinolib_enhance.h
)
target_include_directories(HexerCore PUBLIC ${CMAKE_CURRENT_LIST_DIR})