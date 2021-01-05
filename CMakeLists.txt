cmake_minimum_required(VERSION 3.1)
project(mmesh)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/")
include(ConfigureAllModules) 

__find_trimesh2()
__find_clipper()

add_subdirectory(mmesh)
