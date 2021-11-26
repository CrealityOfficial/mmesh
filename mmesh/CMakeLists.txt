__files_group(${CMAKE_CURRENT_SOURCE_DIR}/mmesh/ SRCS)

if(WIN32)
	list(APPEND DEFS _USE_MATH_DEFINES)
endif()

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

if(TARGET freeImage)
	message(STATUS "build mmesh use freeImage ...........................")
	add_definitions(-DHAVE_FREEIMAGE)
	list(APPEND Libs freeImage)
endif()

__add_real_target(mmesh lib SOURCE ${SRCS} INC ${Incs} LIB ${Libs} DEF ${DEFS} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

if(INSTALL_MMESH)
	INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/mmesh/ DESTINATION include/ FILES_MATCHING PATTERN "*.h")
endif()