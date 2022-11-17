cmake_minimum_required (VERSION 3.19)

project ("Hexer")

set(HEXER_EXPBLAS_PATH )

set(HEXER_SPDLOG_PATH "A:/MeshGeneration/spdlog/include")

set(OPENMESH_PATH "D:/openmesh")
set(OPENMESH_INCLUDE ${OPENMESH_PATH}/include)
set(OPENMESH_LIB ${OPENMESH_PATH}/lib)

set(OPENVOLUMNMESH_PATH_INCLUDE "D:/codes/OpenVolumeMesh/src")
#set(OPENVOLUMNMESH_PATH_LIB "D:/codes/OpenVolumeMesh/build/Build/lib/RelWithDebInfo")
set(OPENVOLUMNMESH_PATH_LIB "D:/codes/OpenVolumeMesh/build/Build/lib/Debug")

add_definitions(-D_USE_MATH_DEFINES)

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}
	${OPENMESH_INCLUDE}
	D:/codes/expblas
	${OPENVOLUMNMESH_PATH_INCLUDE}
)
link_directories(
	${OPENMESH_LIB}
	${OPENVOLUMNMESH_PATH_LIB}
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
	#OpenVolumeMesh.lib
	OpenVolumeMeshd.lib
)
#add_subdirectory("core")
#add_subdirectory ("Hexer")
