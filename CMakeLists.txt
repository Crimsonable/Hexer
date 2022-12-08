cmake_minimum_required (VERSION 3.19)

project ("Hexer")

set(HEXER_EXPBLAS_PATH "D:/codes/expblas")

set(HEXER_SPDLOG_PATH "A:/MeshGeneration/spdlog/include")

set(OPENMESH_PATH "D:/openmesh")
set(OPENMESH_INCLUDE ${OPENMESH_PATH}/include)
set(OPENMESH_LIB ${OPENMESH_PATH}/lib)

set(OPENVOLUMNMESH_PATH_INCLUDE "D:/codes/OpenVolumeMesh/src")
#set(OPENVOLUMNMESH_PATH_LIB "D:/codes/OpenVolumeMesh/build/Build/lib/RelWithDebInfo")
set(OPENVOLUMNMESH_PATH_LIB "D:/codes/OpenVolumeMesh/build/Build/lib/Debug")

add_definitions(-D_USE_MATH_DEFINES)

find_package(OpenVolumeMesh CONFIG REQUIRED)
find_package(OpenMesh CONFIG REQUIRED)

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}
	${OPENVOLUMNMESH_PATH_INCLUDE}
	${HEXER_EXPBLAS_PATH}
)
link_directories(
	${OPENVOLUMNMESH_PATH_LIB}
)

add_executable (Hexer "")
add_library(HexerCore STATIC "")
add_compile_options("/bigobj")

target_link_libraries(HexerCore PRIVATE ${OPENMESH_LIBRARIES})
target_link_libraries(HexerCore PUBLIC OpenVolumeMesh::OpenVolumeMesh)
target_link_libraries(Hexer PRIVATE HexerCore)

add_subdirectory(core)
add_subdirectory(hexer)
add_subdirectory(visual)