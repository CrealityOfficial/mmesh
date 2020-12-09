__files_group(${CMAKE_CURRENT_SOURCE_DIR}/mmesh/ SRCS)

__cgal_include_common_packages()
__cgal_include_package(Straight_skeleton_2)
__cgal_include_package(Polygon)

set(DEFS CGAL_NO_GMP=1 CGAL_NO_MPFR=1)
set(Incs ${CMAKE_CURRENT_SOURCE_DIR})
set(Libs trimesh2 clipper)
__add_real_target(mmesh lib SOURCE ${SRCS} INC ${Incs} LIB ${Libs} DEF ${DEFS})

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/mmesh/ DESTINATION include/ FILES_MATCHING PATTERN "*.h")
set_property(TARGET mmesh PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR})