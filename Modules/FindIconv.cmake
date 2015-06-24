INCLUDE(FindPackageHandleStandardArgs)

FILE(
	GLOB
	ICONV_INCLUDE_PATHS
	$ENV{ICONVDIR}/include
	$ENV{HOME}/include
	/usr/local/include
	/usr/include
	$ENV{ProgramW6432}/iconv*/include
	$ENV{ProgramFiles}/iconv*/include
	$ENV{ProgramW6432}/libiconv*/include
	$ENV{ProgramFiles}/libiconv*/include
	$ENV{ProgramW6432}/GnuWin32/include
	$ENV{ProgramFiles}/GnuWin32/include
)

FILE(
	GLOB
	ICONV_LIBRARY_PATHS
	$ENV{ICONVDIR}/lib
	$ENV{HOME}/lib
	/usr/local/lib
	/usr/lib
	$ENV{ProgramW6432}/iconv*/lib
	$ENV{ProgramFiles}/iconv*/lib
	$ENV{ProgramW6432}/libiconv*/lib
	$ENV{ProgramFiles}/libiconv*/lib
	$ENV{ProgramW6432}/GnuWin32/lib
	$ENV{ProgramFiles}/GnuWin32/lib
)

FIND_PATH(
	ICONV_INCLUDE_DIRS
	NAMES
	iconv.h
	HINTS
	${ICONV_INCLUDE_PATHS}
)

FIND_LIBRARY(
	ICONV_LIBRARIES
	NAMES
	iconv libiconv libiconv_a
	HINTS
	${ICONV_LIBRARY_PATHS}
)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(
	ICONV
	DEFAULT_MSG
	ICONV_INCLUDE_DIRS
	ICONV_LIBRARIES
)

MARK_AS_ADVANCED(
	ICONV_INCLUDE_DIRS
	ICONV_LIBRARIES
) 
