__files_group(${CMAKE_CURRENT_SOURCE_DIR}/mmesh/ SRCS)

if(CGAL_ENABLE)
	include(CGAL)
	
	__cgal_include_package(Straight_skeleton_2)
	__cgal_include_package(Polygon)
	__cgal_include_package(Polygon_mesh_processing)
	__cgal_include_package(Polyhedron)
	__cgal_include_package(Polyhedron_IO)
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
	__cgal_include_package(Surface_mesh_simplification)
	
	#enable cluster point 
	__cgal_include_package(Point_set_processing_3)
	__cgal_include_package(Point_set_3)
	__cgal_include_package(Solver_interface)
	__cgal_include_package(Triangulation_3)
	__cgal_include_package(TDS_3)
	__cgal_include_package(Subdivision_method_3)
	
	
	
	set(DEFS CGAL_NO_GMP=1 CGAL_NO_MPFR=1)
	set(Incs ${CMAKE_CURRENT_SOURCE_DIR})
	add_definitions(-DUSE_CGAL_REPAIR_MESH)
	add_definitions(-DCGAL_EIGEN3_ENABLED)
	add_definitions(-DUSE_CGAL)
endif()

include_directories(${VCG_INCLUDES})
include_directories(${EIGEN_INCLUDES})
__enable_openmp()

if(WIN32)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()

set(Libs trimesh2 clipper)
if(HAVE_BOOST_ERIALIZATION)
	__files_group(${CMAKE_CURRENT_SOURCE_DIR}/mmeshtest/ SRCS_TEST)
	source_group(cereal FILES ${SRCS_TEST})
	list(APPEND SRCS ${SRCS_TEST})
	list(APPEND Libs boost_serialization gtest)
endif()

__add_real_target(mmesh lib SOURCE ${SRCS} INC ${Incs} LIB ${Libs} DEF ${DEFS} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

if(INSTALL_MMESH)
	INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/mmesh/ DESTINATION include/ FILES_MATCHING PATTERN "*.h")
endif()