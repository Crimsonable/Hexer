cmake_minimum_required (VERSION 3.19)

project ("Hexer")

set(OPENMESH_INCLUDE "D:/openmesh/OpenMesh 9.1/include")
set(OPENMESH_LIB "D:/openmesh/OpenMesh 9.1/lib")

add_definitions(-D_USE_MATH_DEFINES)

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}
	${OPENMESH_INCLUDE}
	D:/codes/expblas
)
link_directories(
	${OPENMESH_LIB}
)
add_executable (Hexer "")

add_subdirectory(core)
add_subdirectory(Hexer)

#file(GLOB_RECURSE core_sources RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*core/*.cpp" "*core/*.h" "*core/*.hpp")
#file(GLOB_RECURSE hexrer_sources RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*hexer/*.cpp" "*hexer/*.h" "*hexer/*.hpp")
#file(GLOB_RECURSE ALL_HEAD "*.hpp" "*.h")
#message(${core_sources})

#add_executable (Hexer ${core_sources} ${hexrer_sources})
target_link_libraries(
	Hexer
	OpenMeshTools.lib
    OpenMeshCore.lib
)
#add_subdirectory("core")
#add_subdirectory ("Hexer")
