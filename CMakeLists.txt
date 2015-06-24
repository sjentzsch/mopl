cmake_minimum_required(VERSION 2.8.11)

project(mopl)

set(VERSION_MAJOR 0)
set(VERSION_MINOR 0)
set(VERSION_PATCH 1)
set(VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

set(CMAKE_VERBOSE_MAKEFILE 1)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Modules)

set(Boost_ADDITIONAL_VERSIONS "1.56.0" "1.56" "1.55.0" "1.55" "1.54.0" "1.54" "1.53.0" "1.53" "1.52.0" "1.52" "1.51.0" "1.51" "1.50.0" "1.50" "1.49.0" "1.49" "1.48.0" "1.48" "1.47.0" "1.47")

if(CMAKE_SIZEOF_VOID_P EQUAL 4)
	add_definitions(-DEIGEN_DONT_ALIGN)
endif(CMAKE_SIZEOF_VOID_P EQUAL 4)

include(CheckCXXCompilerFlag)

if(WIN32)
	add_definitions(
		-D_SCL_SECURE_NO_WARNINGS
		-D_USE_MATH_DEFINES
		-D_WIN32_WINNT=0x400
		-DNOMINMAX
	)
else()
	check_cxx_compiler_flag("-std=c++0x" HAS_CXX_COMPILER_FLAG_STDCXX0X)
	if(HAS_CXX_COMPILER_FLAG_STDCXX0X)
	    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
	endif()
endif()

add_subdirectory(src)
add_subdirectory(src/rlDamaDemoGUI)
add_subdirectory(src/rlDamaRun)

# create symlink to models
execute_process(
	COMMAND ${CMAKE_COMMAND} -E create_symlink 
	${CMAKE_CURRENT_SOURCE_DIR}/data
	${CMAKE_CURRENT_BINARY_DIR}/data
)

enable_testing()
add_subdirectory(tests/testMoplPlan)
add_subdirectory(tests/testIK)
