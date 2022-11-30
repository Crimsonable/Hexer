cmake_minimum_required (VERSION 3.19)

project ("Visual")

set(HEXER_EXPBLAS_PATH "D:/codes/expblas")
set(HEXER_SPDLOG_PATH "A:/MeshGeneration/spdlog/include")

set(OPENMESH_PATH "D:/openmesh")

set(OPENVOLUMNMESH_PATH_INCLUDE "D:/codes/OpenVolumeMesh/src")
#set(OPENVOLUMNMESH_PATH_LIB "D:/codes/OpenVolumeMesh/build/Build/lib/RelWithDebInfo")
set(OPENVOLUMNMESH_PATH_LIB "D:/codes/OpenVolumeMesh/build/Build/lib/Debug")

#set(GLFW_PATH "D:/codes/opengl/glfw")
#set(OPENGL_PATH "D:/codes/opengl")

find_package(glad CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(assimp CONFIG REQUIRED)
find_package(openmesh CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)

add_definitions(-D_USE_MATH_DEFINES)

include_directories(
	${CMAKE_SOURCE_DIR}
	#${OPENMESH_PATH}/include
	${HEXER_EXPBLAS_PATH}
	${OPENVOLUMNMESH_PATH}/include
)
link_directories(
	#${OPENMESH_PATH}/lib
	${OPENVOLUMNMESH_PATH}/lib
)
add_compile_options("/bigobj")

add_executable (Visual "")
target_sources(
	Visual
	PRIVATE
		visual.cpp
		shader.cpp
		mesh.cpp
		model.cpp
		camera.cpp
		glad.c
	PUBLIC
		meshLoader.h
		mesh.h
		base.h
		shader.h
		camera.h
		model.h
)

target_link_libraries(Visual PRIVATE glad::glad)
target_link_libraries(Visual PRIVATE glfw)
target_link_libraries(Visual PRIVATE assimp::assimp)
target_link_libraries(Visual PRIVATE OpenGL32.lib)
target_link_libraries(Visual PRIVATE OpenMeshCore)
target_link_libraries(Visual PRIVATE OpenMeshTools)
target_link_libraries(Visual PRIVATE glm::glm)

#target_link_libraries(
#	Visual
#	OpenMeshTools.lib
#   OpenMeshCore.lib
#	#OpenVolumeMesh.lib
#	OpenVolumeMeshd.lib
#	OpenGL32.lib
#)
