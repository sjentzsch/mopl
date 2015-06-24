project(testIK)

find_package(LibXml2 REQUIRED)
find_package(RL COMPONENTS MDL REQUIRED)


include_directories(
	BEFORE
	${CMAKE_CURRENT_SOURCE_DIR}/../../src
	${EIGEN_INCLUDE_DIRS}
	${LIBXML2_INCLUDE_DIRS}
)

add_executable(
	testIK
	testIK.cpp
)

target_link_libraries(
	testIK
	moplplan
	${RL_LIBRARIES}
	${LIBXML2_LIBRARIES}
)

add_test(
	NAME testIK1
	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/../..
	COMMAND ${CMAKE_CURRENT_BINARY_DIR}/testIK  
)
