CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(rlDamaRun)

FIND_PACKAGE(Ice)
FIND_PACKAGE(Boost COMPONENTS system REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)
FIND_PACKAGE(LibXml2 REQUIRED)
FIND_PACKAGE(Boost REQUIRED)
FIND_PATH(FRI_LIBRARY_DIR
  NAMES 
  Linux/x64/release/lib/libFastResearchInterfaceLibrary.a
  Linux/x64/release/lib/FastResearchInterfaceLibrary.a
  PATHS 
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../../../gaschler/FRILibrary
  DOC 
  "Fast Research Interface Library for Kuka LWR4 robot"
)

find_package(RL COMPONENTS KIN MDL HAL REQUIRED)

INCLUDE_DIRECTORIES(
	BEFORE
	${CMAKE_CURRENT_SOURCE_DIR}/..
	${Boost_INCLUDE_DIR}
	${EIGEN_INCLUDE_DIRS}
	${LIBXML2_INCLUDE_DIRS}
)

if(ICE_FOUND)
	INCLUDE_DIRECTORIES(
		BEFORE
		${ICE_INCLUDE_DIR}
	)
	SET(
		SRCS
		rlDamaRun.cpp
		DamaExecutor.h
		DamaExecutor.cpp
	)

	ICE_WRAP_CPP_2(
		ICE_SOURCES
		${CMAKE_CURRENT_SOURCE_DIR}
		meka-robot.ice
	)
	
	ADD_EXECUTABLE(
		rlDamaMekaRun
		${ICE_SOURCES}
		${SRCS}
	)
	
	LINK_DIRECTORIES(
		${Boost_LIBRARY_DIRS}
	)
	
	TARGET_LINK_LIBRARIES(
		rlDamaMekaRun
		${RL_LIBRARIES}
		${Boost_LIBRARIES}
		${LIBXML2_LIBRARIES}
		${ICE_LIBRARIES}
	)
else(ICE_FOUND)
	message(WARNING "ZeroC Ice not found, skipping Meka robot execution client")
endif(ICE_FOUND)

ADD_EXECUTABLE(
	rlDamaCoachRun
	rlDamaKukaFRIRun.cpp
	DamaKukaFRIExecutor.h
	DamaKukaFRIExecutor.cpp
)
SET_TARGET_PROPERTIES(
	rlDamaCoachRun
	PROPERTIES 
	COMPILE_DEFINITIONS	USE_REAL_ROBOT=0 
)
TARGET_LINK_LIBRARIES(
	rlDamaCoachRun
	${RL_LIBRARIES}
	${Boost_LIBRARIES}
	${LIBXML2_LIBRARIES}
)
	
if(FRI_LIBRARY_DIR)
	ADD_DEFINITIONS(
		-DFRI_LIBRARY_DIR="${FRI_LIBRARY_DIR}"
	)
	INCLUDE_DIRECTORIES(
		${FRI_LIBRARY_DIR}/include
	)
	LINK_DIRECTORIES(
		${FRI_LIBRARY_DIR}/Linux/x64/release/lib
	)
	ADD_EXECUTABLE(
		rlDamaKukaFRIRun
		rlDamaKukaFRIRun.cpp
		DamaKukaFRIExecutor.h
		DamaKukaFRIExecutor.cpp
		KukaFRI.cpp
	)
	SET_TARGET_PROPERTIES(
		rlDamaKukaFRIRun
		PROPERTIES 
		COMPILE_DEFINITIONS	USE_REAL_ROBOT=1 
	)
	TARGET_LINK_LIBRARIES(
		rlDamaKukaFRIRun
		${RL_LIBRARIES}
		${Boost_LIBRARIES}
		${LIBXML2_LIBRARIES}
		FastResearchInterfaceLibrary pthread dl rt 
		#####stdc++ m rt
	)
else()
	message(WARNING "FRI Library x64/release not found, skipping Kuka FRI driver")
endif()
