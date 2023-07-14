cmake_minimum_required (VERSION 3.19)

project ("Hexer")

#set(HEXER_EXPBLAS_PATH "A:/MeshGeneration")
#set(cinolib_DIR "A:/MeshGeneration/cinolib")

set(cinolib_DIR "D:/codes/cinolib")
set(HEXER_EXPBLAS_PATH "D:/codes/expblas")

#set(cinolib_DIR "H:/toys/cinolib")
#set(HEXER_EXPBLAS_PATH "H:/toys/expblas")

add_definitions(-D_USE_MATH_DEFINES)
set(CINOLIB_USES_OPENGL_GLFW_IMGUI ON)
set(CINOLIB_USES_VTK ON)
set(CINOLIB_USES_TETGEN ON)

#find_package(OpenVolumeMesh CONFIG REQUIRED)
#find_package(OpenMesh CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(cinolib CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(range-v3 CONFIG REQUIRED)
#find_package(Boost REQUIRED Graph)

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}
	${OPENVOLUMNMESH_PATH_INCLUDE}
	${HEXER_EXPBLAS_PATH}
)
link_directories(
	${OPENVOLUMNMESH_PATH_LIB}
)

add_compile_options("/openmp")
add_compile_options("/bigobj")
if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
	message("Using Debug Mode")
	add_compile_definitions(__DEBUG)
endif()



add_library(HexerCore STATIC "")
target_link_libraries(HexerCore PUBLIC Eigen3::Eigen)
target_link_libraries(HexerCore PUBLIC cinolib)
target_link_libraries(HexerCore PUBLIC glfw)
target_link_libraries(HexerCore PUBLIC OpenGL32.lib)
target_link_libraries(HexerCore PUBLIC spdlog::spdlog spdlog::spdlog_header_only)
target_link_libraries(HexerCore PUBLIC range-v3 range-v3-meta range-v3::meta range-v3-concepts)
#target_link_libraries(HexerCore PUBLIC Boost::boost Boost::Graph)

add_subdirectory(core)

add_executable (Smoother "")
target_link_libraries(Smoother PRIVATE HexerCore)
add_subdirectory("./hexer/smoother")

add_executable(GlobalOrientation "")
target_link_libraries(GlobalOrientation PRIVATE HexerCore)
add_subdirectory("./hexer/globalOrientation")

add_executable (SmoothEnergy "")
target_link_libraries(SmoothEnergy PRIVATE HexerCore)
add_subdirectory("./hexer/GaussianSmoothFacetNormals")

#add_subdirectory(hexer)
#add_subdirectory(visual)