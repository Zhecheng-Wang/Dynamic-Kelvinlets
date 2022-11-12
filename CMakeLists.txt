cmake_minimum_required(VERSION 3.16)
project(kelvinlets LANGUAGES CXX)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

if(NOT CMAKE_BUILD_TYPE)
	message(STATUS "No build type selected, default to Release")
	set(CMAKE_BUILD_TYPE "Release")
endif()

# Find Polyscope
find_package(Polyscope REQUIRED)

file(GLOB SRCFILES src/*.h src/*.cpp)

add_executable(${PROJECT_NAME} ${SRCFILES})
target_include_directories( ${PROJECT_NAME} PUBLIC)

# Libigl
option(LIBIGL_GLFW "Build target igl::glfw" OFF)
include(libigl)

# Use C++14
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 14)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD_REQUIRED ON)

target_link_libraries(${PROJECT_NAME} igl::core polyscope)