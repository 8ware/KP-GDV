# - Try to find OpenNI
# Once done, this will define
#
#  OpenNI_FOUND - system has OpenNI
#  OpenNI_INCLUDE_DIRS - the OpenNI include directories
#  OpenNI_LIBRARIES - link these to use OpenNI

IF (UNIX)

  IF(NOT OPEN_NI_ROOT)
    IF(EXISTS "/usr/include/ni")
  	  SET(OPEN_NI_ROOT "/usr")
    ELSEIF(EXISTS "/usr/local/include/ni")
  	  SET(OPEN_NI_ROOT "/usr/local")
    ELSE()
      MESSAGE("OPEN_NI_ROOT not set. Continuing anyway..")
    ENDIF()
  ENDIF()

	# Use pkg-config to get hints about paths
	libfind_pkg_check_modules(OpenNI_PKGCONF libOpenNI)
	# Include dir
	find_path(OpenNI_INCLUDE_DIR
	  NAMES XnOpenNI.h
	  PATHS ${OpenNI_PKGCONF_INCLUDE_DIRS} ${OPEN_NI_ROOT}/include/ni
	)
	# Finally the library itself
	find_library(OpenNI_LIBRARY
	  NAMES OpenNI
	  PATHS ${OpenNI_PKGCONF_LIBRARY_DIRS} ${OPEN_NI_ROOT}/lib
	)
ELSEIF (WIN32)
	find_path(OpenNI_INCLUDE_DIR
	  NAMES XnOpenNI.h
	  PATHS "${OPEN_NI_ROOT}/Include" "C:/Program Files (x86)/OpenNI/Include" "C:/Program Files/OpenNI/Include" ${CMAKE_INCLUDE_PATH}
	)
	# Finally the library itself
	find_library(OpenNI_LIBRARY
	  NAMES OpenNI
	  PATHS "${OPEN_NI_ROOT}/Lib" "C:/Program Files (x86)/OpenNI/Lib" "C:/Program Files/OpenNI/Lib" ${CMAKE_LIB_PATH}
	)
ENDIF()
