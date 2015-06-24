PROJECT(moplPlan)

FIND_PACKAGE(Cgal REQUIRED)
FIND_PACKAGE(Coin REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)
FIND_PACKAGE(LibXml2 REQUIRED)
FIND_PACKAGE(Boost COMPONENTS system REQUIRED)
FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(Qt4 REQUIRED)
FIND_PACKAGE(Solid REQUIRED)
FIND_PACKAGE(SoQt REQUIRED)

find_package(RL 0.6.3 COMPONENTS KIN MDL SG PLAN REQUIRED)


ADD_DEFINITIONS(
	-DCGAL_DISABLE_ROUNDING_MATH_CHECK
	${COIN_DEFINITIONS}
)

INCLUDE_DIRECTORIES(
	BEFORE
	${Boost_INCLUDE_DIR}
	${CGAL_INCLUDE_DIRS}
	${COIN_INCLUDE_DIRS}
	${EIGEN_INCLUDE_DIRS}
	${LIBXML2_INCLUDE_DIRS}
)

LINK_DIRECTORIES(
	${Boost_LIBRARY_DIRS}
)

SET(
	HDRS
	DamaRrtGoalBias.h
	DamaRrtAction.h
	DamaRrtCon.h
	DamaModel.h
	DamaPrim.h
	DamaPrimPickup.h
	DamaPrimPush.h
	DamaPrimPushInterior.h
	DamaPrimPushExterior.h
	DamaPrimPushFrontal.h
	DamaPrimPushMobile.h
	DamaPrimTransferRigid.h
	DamaPrimTransit.h
	DamaRrt.h
	DamaSampler.h
	DamaSupportSurface.h
	Timer.h
	XmlFactory.h
)

SET(
	SRCS
	DamaRrtGoalBias.cpp
	DamaRrtAction.cpp
	DamaRrtCon.cpp
	DamaModel.cpp
	DamaPrimPickup.cpp
	DamaPrimPush.cpp
	DamaPrimPushInterior.cpp
	DamaPrimPushExterior.cpp
	DamaPrimPushFrontal.cpp
	DamaPrimPushMobile.cpp
	DamaPrimTransferRigid.cpp
	DamaPrimTransit.cpp
	DamaRrt.cpp
	DamaSampler.cpp
	DamaSupportSurface.cpp
	XmlFactory.cpp
)

ADD_LIBRARY(
	moplplan
	SHARED
	${HDRS}
	${SRCS}
)

TARGET_LINK_LIBRARIES(
	moplplan
	${RL_LIBRARIES}
	${Boost_LIBRARIES}
	${CGAL_LIBRARIES}
	${COIN_LIBRARIES}
	${LIBXML2_LIBRARIES}
)
