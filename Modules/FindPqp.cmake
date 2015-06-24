INCLUDE(FindPackageHandleStandardArgs)

FILE(
	GLOB
	PQP_INCLUDE_PATHS
	$ENV{PQPDIR}/include
	$ENV{HOME}/include
	/usr/local/include
	/usr/include
	$ENV{SystemDrive}/pqp*/include
	$ENV{ProgramW6432}/pqp*/include
	$ENV{ProgramFiles}/pqp*/include
)

FILE(
	GLOB
	PQP_LIBRARY_PATHS
	$ENV{PQPDIR}/lib
	$ENV{HOME}/lib
	/usr/local/lib
	/usr/lib
	$ENV{SystemDrive}/pqp*/lib
	$ENV{ProgramW6432}/pqp*/lib
	$ENV{ProgramFiles}/pqp*/lib
)

FIND_PATH(
	PQP_INCLUDE_DIRS
	NAMES
	PQP.h
	HINTS
	${PQP_INCLUDE_PATHS}
)

FIND_LIBRARY(
	PQP_LIBRARIES
	NAMES
	PQP
	HINTS
	${PQP_LIBRARY_PATHS}
)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(
	PQP
	DEFAULT_MSG
	PQP_INCLUDE_DIRS
	PQP_LIBRARIES
)

MARK_AS_ADVANCED(
	PQP_INCLUDE_DIRS
	PQP_LIBRARIES
) 
