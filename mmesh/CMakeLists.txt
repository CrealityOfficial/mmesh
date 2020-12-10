__files_group(${CMAKE_CURRENT_SOURCE_DIR}/mmesh/ SRCS)

__cgal_include_common_packages()
__cgal_include_package(Straight_skeleton_2)
__cgal_include_package(Polygon)
__cgal_include_package(Polygon_mesh_processing)
__cgal_include_package(Random_numbers)
__cgal_include_package(Box_intersection_d) 
__cgal_include_package(AABB_tree)
__cgal_include_package(Spatial_searching)
__cgal_include_package(Spatial_sorting)
__cgal_include_package(Generator)
__cgal_include_package(Union_find)
__cgal_include_package(Modifier)
__cgal_include_package(Triangulation_2)
__cgal_include_package(TDS_2)
__cgal_include_package(Surface_mesh)

set(DEFS CGAL_NO_GMP=1 CGAL_NO_MPFR=1)
set(Incs ${CMAKE_CURRENT_SOURCE_DIR})
add_definitions(-DUSE_CGAL_REPAIR_MESH)

if(WIN32)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()


set(Libs trimesh2 clipper)
__add_real_target(mmesh lib SOURCE ${SRCS} INC ${Incs} LIB ${Libs} DEF ${DEFS})

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/mmesh/ DESTINATION include/ FILES_MATCHING PATTERN "*.h")
set_property(TARGET mmesh PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR})