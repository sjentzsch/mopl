project(testMoplPlan)

find_package(Boost COMPONENTS system REQUIRED)
find_package(Cgal REQUIRED)
find_package(Coin REQUIRED)
find_package(Eigen REQUIRED)
find_package(LibXml2 REQUIRED)
find_package(Solid REQUIRED)
find_package(RL COMPONENTS KIN MDL SG PLAN REQUIRED)

add_definitions(
	-DCGAL_DISABLE_ROUNDING_MATH_CHECK
	${COIN_DEFINITIONS}
	-DHAVE_SOLID
)

include_directories(
	BEFORE
	${CMAKE_CURRENT_SOURCE_DIR}/../../src
	${Boost_INCLUDE_DIR}
	${CGAL_INCLUDE_DIRS}
	${COIN_INCLUDE_DIRS}
	${EIGEN_INCLUDE_DIRS}
	${LIBXML2_INCLUDE_DIRS}
)

add_executable(
	testMoplPlan
	testDamaPlan.cpp
)

link_directories(
	${Boost_LIBRARY_DIRS}
	${SOLID_INCLUDE_DIRS}
)
target_link_libraries(
	testMoplPlan
	moplplan
	${RL_LIBRARIES}
	${Boost_LIBRARIES}
	${CGAL_LIBRARIES}
	${COIN_LIBRARIES}
	${LIBXML2_LIBRARIES}
	${SOLID_LIBRARIES}
)

add_test(
	NAME testMoplMekaSimplePush
	COMMAND ${CMAKE_CURRENT_BINARY_DIR}/../../src/rlDamaDemoGUI/rlDamaDemoGUI ${CMAKE_CURRENT_SOURCE_DIR}/../../data/tasks-test/meka-simple-push.xml
)

add_test(
	NAME testMoplKukaTransfer3Objects
	COMMAND ${CMAKE_CURRENT_BINARY_DIR}/../../src/rlDamaDemoGUI/rlDamaDemoGUI ${CMAKE_CURRENT_SOURCE_DIR}/../../data/tasks-test/kuka-transfer-3-objects.xml
)

add_test(
	NAME testMoplMobileRobotMoveBlockedObject
	COMMAND ${CMAKE_CURRENT_BINARY_DIR}/../../src/rlDamaDemoGUI/rlDamaDemoGUI ${CMAKE_CURRENT_SOURCE_DIR}/../../data/tasks-test/mobile-robot-move-blocked-object.xml
)
