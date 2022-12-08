cmake_minimum_required (VERSION 3.19)

project ("Hexer")

set(HEXER_EXPBLAS_PATH "D:/codes/expblas")

set(HEXER_SPDLOG_PATH "A:/MeshGeneration/spdlog/include")
set(OpenVolumeMesh_DIR "D:/codes/OVM/lib/cmake/OpenVolumeMesh")

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