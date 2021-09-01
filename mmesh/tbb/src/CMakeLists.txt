include(CheckCXXCompilerFlag)
include(CheckCXXSourceRuns)

file(GLOB tbb_src "${CMAKE_CURRENT_SOURCE_DIR}/src/tbb/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/old/*.cpp")
list(APPEND tbb_src ${CMAKE_CURRENT_SOURCE_DIR}/src/rml/client/rml_tbb.cpp)
file(GLOB to_remove "${CMAKE_CURRENT_SOURCE_DIR}/src/old/test*.cpp")
list(REMOVE_ITEM tbb_src ${to_remove})

configure_file (build/version_string.ver.in version_string.ver @ONLY)

if (UNIX)
	add_definitions (-DUSE_PTHREAD)
	
	check_cxx_compiler_flag ("-std=c++11" SUPPORTS_STDCXX11)
	if (SUPPORTS_STDCXX11)
	set (CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
	endif ()
	
	check_cxx_compiler_flag ("-mrtm -Werror" SUPPORTS_MRTM)
	if (SUPPORTS_MRTM)
	set (CMAKE_CXX_FLAGS "-mrtm ${CMAKE_CXX_FLAGS}")
	endif ()
elseif(WIN32)
	if (MSVC)
		enable_language(ASM_MASM)
		set(CMAKE_CXX_FLAGS "/GS- /Zc:wchar_t /Zc:forScope /DUSE_WINTHREAD ${CMAKE_CXX_FLAGS}")
		set(CMAKE_CXX_FLAGS "/D_CRT_SECURE_NO_DEPRECATE /D_WIN32_WINNT=0x0600 ${CMAKE_CXX_FLAGS}")
		check_cxx_compiler_flag ("/volatile:iso" SUPPORTS_VOLATILE_FLAG)
		if (SUPPORTS_VOLATILE_FLAG)
			set(CMAKE_CXX_FLAGS "/volatile:iso ${CMAKE_CXX_FLAGS}")
		endif ()
		set(CMAKE_CXX_FLAGS "/wd4267 /wd4800 /wd4146 /wd4244 /wd4577 /wd4018 ${CMAKE_CXX_FLAGS}")
		if (NOT CMAKE_SIZEOF_VOID_P)
			message(FATAL_ERROR "'CMAKE_SIZEOF_VOID_P' is undefined. Please delete your build directory and rerun CMake again!")
		endif()
		
		if (CMAKE_SIZEOF_VOID_P EQUAL 8)
			list(APPEND tbb_src src/tbb/intel64-masm/atomic_support.asm
			src/tbb/intel64-masm/itsx.asm src/tbb/intel64-masm/intel64_misc.asm)
			list(APPEND tbbmalloc_src src/tbb/intel64-masm/atomic_support.asm)
			set(CMAKE_ASM_MASM_FLAGS "/DEM64T=1 ${CMAKE_ASM_MASM_FLAGS}")
		else()
			list(APPEND tbb_src src/tbb/ia32-masm/atomic_support.asm
			src/tbb/ia32-masm/itsx.asm src/tbb/ia32-masm/lock_byte.asm)
			# Enable SAFESEH feature for assembly (x86 builds only).
			set(CMAKE_ASM_MASM_FLAGS "/safeseh ${CMAKE_ASM_MASM_FLAGS}")
		endif()
	elseif (MINGW)
		add_definitions(-DUSE_WINTHREAD)
		add_definitions(-D_WIN32_WINNT=0x0502)
		set(CMAKE_CXX_FLAGS "-mthreads ${CMAKE_CXX_FLAGS}")
	endif ()
endif()

if (MSVC)
	set(ENABLE_RTTI "/EHsc /GR ")
	set(DISABLE_RTTI "/EHs- /GR- ")
elseif (UNIX)
	set(ENABLE_RTTI "-frtti -fexceptions ")
	set(DISABLE_RTTI "-fno-rtti -fno-exceptions ")
endif ()

set(INCS ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_BINARY_DIR})
__add_real_target(tbb lib SOURCE ${tbb_src}
						  INC ${INCS} ${CMAKE_CURRENT_SOURCE_DIR}/src/
						  DEF __TBB_SOURCE_DIRECTLY_INCLUDED
						  INTERFACE ${INCS})
						  
set_property(TARGET tbb APPEND PROPERTY COMPILE_DEFINITIONS "__TBB_BUILD=1")
set_property(TARGET tbb APPEND_STRING PROPERTY COMPILE_FLAGS ${ENABLE_RTTI})

if (UNIX AND NOT APPLE)
	target_link_libraries(tbb PUBLIC pthread dl)
endif()
if (MSVC)
	target_compile_definitions(tbb PUBLIC __TBB_NO_IMPLICIT_LINKAGE=1)
endif()