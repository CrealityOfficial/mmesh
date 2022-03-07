__files_group(${CMAKE_CURRENT_SOURCE_DIR}/mmesh/ SRCS)

if(WIN32)
	list(APPEND DEFS _USE_MATH_DEFINES)
endif()

if(WIN32)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()
set(LIBS trimesh2 OpenMP::OpenMP_CXX)
set(INCS )

if(TARGET freeImage)
	message(STATUS "build mmesh use freeImage ...........................")
	add_definitions(-DHAVE_FREEIMAGE)
	list(APPEND LIBS freeImage)
endif()

__add_real_target(mmesh lib SOURCE ${SRCS} 
							INC ${INCS}
							LIB ${LIBS}
							DEF ${DEFS}
							INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
							)
							
if(INSTALL_MMESH)
	INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/mmesh/ DESTINATION include/ FILES_MATCHING PATTERN "*.h")
endif()
