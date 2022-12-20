cmake_minimum_required (VERSION 3.19)

project ("Hexer")

set(HEXER_EXPBLAS_PATH "D:/codes/expblas")
set(HEXER_SPDLOG_PATH "A:/MeshGeneration/spdlog/include")
set(OpenVolumeMesh_DIR "D:/codes/OVM/lib/cmake/OpenVolumeMesh")
set(cinolib_DIR "D:/codes/cinolib")

add_definitions(-D_USE_MATH_DEFINES)
add_definitions(-DCINOLIB_USES_OPENGL_GLFW_IMGUI)

find_package(OpenVolumeMesh CONFIG REQUIRED)
find_package(OpenMesh CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(cinolib CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)

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
target_link_libraries(HexerCore PUBLIC Eigen3::Eigen)
target_link_libraries(HexerCore PUBLIC cinolib)
target_link_libraries(HexerCore PUBLIC glfw)
target_link_libraries(Hexer PRIVATE HexerCore)

add_subdirectory(core)
add_subdirectory(hexer)
add_subdirectory(visual)