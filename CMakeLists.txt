cmake_minimum_required (VERSION 3.19)

project ("Hexer")

set(HEXER_EXPBLAS_PATH "A:/MeshGeneration")

set(HEXER_SPDLOG_PATH "A:/MeshGeneration/spdlog/include")

set(OPENVOLUMNMESH_PATH "A:/MeshGeneration/OpenVolumeMesh")
#set(OPENVOLUMNMESH_PATH_LIB "D:/codes/OpenVolumeMesh/build/Build/lib/RelWithDebInfo")
set(OPENVOLUMNMESH_PATH_LIB "A:/MeshGeneration/OVM/Build/lib/Debug")

add_definitions(-D_USE_MATH_DEFINES)

find_package(openmesh CONFIG REQUIRED)


include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}
	${HEXER_EXPBLAS_PATH}
	${OPENMESH_INCLUDE_DIRS}
	${OPENVOLUMNMESH_PATH}/src
)
link_directories(
	${OPENVOLUMNMESH_PATH_LIB}
)


add_compile_options("/bigobj")

add_executable (Hexer "")
add_library(HexerCore STATIC "")
add_subdirectory(core)
add_subdirectory(hexer)
add_subdirectory(visual)

target_link_libraries(
	Hexer
	${OPENMESH_LIBRARIES}
	#OpenVolumeMesh.lib
	OpenVolumeMeshd.lib
	HexerCore
)
