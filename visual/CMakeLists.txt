cmake_minimum_required (VERSION 3.19)

project("Visual")

find_package(glad CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(assimp CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)

add_definitions(-D_USE_MATH_DEFINES)

include_directories(
	${CMAKE_SOURCE_DIR}
	${HEXER_EXPBLAS_PATH}
)
link_directories(
	#${OPENMESH_PATH}/lib
	${OPENVOLUMNMESH_PATH_LIB}
)
add_compile_options("/bigobj")

add_executable (Visual "")
target_sources(
	Visual
	PRIVATE
		meshLoader.cpp
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
target_link_libraries(Visual PRIVATE glm::glm)
target_link_libraries(Visual PRIVATE HexerCore)
#target_link_libraries(Visual PRIVATE OpenVolumeMeshd.lib)