cmake_minimum_required(VERSION 3.1)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/")
include(ConfigureAllModules) 

project(mmesh)

add_subdirectory(trimesh2/external/ trimesh2)
add_subdirectory(mmesh)

__configure_all()