INCLUDE(FindPackageHandleStandardArgs)

FILE(
	GLOB
	EIGEN_INCLUDE_PATHS
	$ENV{EIGENDIR}
	$ENV{HOME}/include
	/usr/local/include/eigen3
	/usr/local/include/eigen*
	/usr/local/include
	/usr/include/eigen3
	/usr/include/eigen*
	/usr/include
	/opt/local/include/eigen*
	$ENV{ProgramW6432}/eigen*/include/eigen3
	$ENV{ProgramFiles}/eigen*/include/eigen3
	$ENV{ProgramW6432}/eigen*
	$ENV{ProgramFiles}/eigen*
)

FIND_PATH(
	EIGEN_INCLUDE_DIRS
	NAMES
	Eigen/Core
	HINTS
	${EIGEN_INCLUDE_PATHS}
)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(
	Eigen
	DEFAULT_MSG
	EIGEN_INCLUDE_DIRS
)

MARK_AS_ADVANCED(
	EIGEN_FOUND
	EIGEN_INCLUDE_DIRS
) 
