################################################################################
# Required CMake version
################################################################################

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12)

SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake/")

################################################################################
# Settings
################################################################################

SET(EXTERNAL_INCLUDE_DIRS)

SET(LIBKINDRV_TARGET_NAME libkindrv)
SET(LIBKINDRV_INCLUDE_DIR "external/")
SET(LIBKINDRV_SUFFIXED_INCLUDE_DIR "${LIBKINDRV_INCLUDE_DIR}${LIBKINDRV_TARGET_NAME}/")
SET(LIBKINDRV_SUFFIXED_SOURCE_DIR ${LIBKINDRV_SUFFIXED_INCLUDE_DIR})
LIST(APPEND EXTERNAL_INCLUDE_DIRS PUBLIC ${LIBKINDRV_INCLUDE_DIR})

SET(DIRENTWIN_TARGET_NAME dirent-win)
SET(DIRENTWIN_INCLUDE_DIR "external/")
SET(DIRENTWIN_SUFFIXED_INCLUDE_DIR "${DIRENTWIN_INCLUDE_DIR}${DIRENTWIN_TARGET_NAME}/")
SET(DIRENTWIN_SUFFIXED_SOURCE_DIR ${DIRENTWIN_SUFFIXED_INCLUDE_DIR})
LIST(APPEND EXTERNAL_INCLUDE_DIRS PUBLIC ${DIRENTWIN_INCLUDE_DIR})

SET(RAPIDJSON_TARGET_NAME rapidjson)
SET(RAPIDJSON_INCLUDE_DIR "external/rapidjson/include/")
SET(RAPIDJSON_SUFFIXED_INCLUDE_DIR "${RAPIDJSON_INCLUDE_DIR}${RAPIDJSON_TARGET_NAME}/")
SET(RAPIDJSON_SUFFIXED_SOURCE_DIR ${RAPIDJSON_SUFFIXED_INCLUDE_DIR})
LIST(APPEND EXTERNAL_INCLUDE_DIRS PUBLIC ${RAPIDJSON_INCLUDE_DIR})

SET(EASYLOGGING_TARGET_NAME easylogging)
SET(EASYLOGGING_INCLUDE_DIR "external/easylogging/src/")
SET(EASYLOGGING_SUFFIXED_INCLUDE_DIR "${EASYLOGGING_INCLUDE_DIR}${EASYLOGGING_TARGET_NAME}/")
SET(EASYLOGGING_SUFFIXED_SOURCE_DIR ${EASYLOGGING_SUFFIXED_INCLUDE_DIR})
LIST(APPEND EXTERNAL_INCLUDE_DIRS PUBLIC ${EASYLOGGING_INCLUDE_DIR})

SET(KINJO_TARGET_NAME kinjo)
SET(KINJO_INCLUDE_DIR "include/")
SET(KINJO_SUFFIXED_INCLUDE_DIR "${KINJO_INCLUDE_DIR}${KINJO_TARGET_NAME}/")
SET(KINJO_SUFFIXED_SOURCE_DIR "src/")

# The visual studio solution name.
PROJECT(${KINJO_TARGET_NAME})

# Include recursive file functions. 
SET(KINJO_COMMON_FUNCTIONS_FILE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake/common.cmake")
INCLUDE(${KINJO_COMMON_FUNCTIONS_FILE_PATH})

################################################################################
# OpenNI
################################################################################

#OPEN_NI_ROOT
FIND_PACKAGE(OpenNI REQUIRED)

################################################################################
# OpenCV
################################################################################

#OpenCV_DIR
#SET(OpenCV_STATIC ON)
SET(OpenCV_CUDA OFF)
FIND_PACKAGE(OpenCV REQUIRED)

################################################################################
# libusb-1.0
################################################################################

# Add our find script.
#LIBUSB_1_ROOT
FIND_PACKAGE(libusb-1.0 REQUIRED)

################################################################################
# Build settings.
################################################################################
SET(COMPILE_OPTIONS)
SET(LINK_LIBS)

IF(MSVC)
    LIST(APPEND COMPILE_OPTIONS "PUBLIC;/W4")
ELSE()
    LIST(APPEND COMPILE_OPTIONS "PUBLIC;-Wall")
    LIST(APPEND COMPILE_OPTIONS "PUBLIC;-std=c++11")
    LIST(APPEND LINK_LIBS "PUBLIC;pthread")
ENDIF()

################################################################################
# libkindrv
################################################################################

# Add all the include files in all recursive subdirectories and group them accordingly.
append_recursive_files_add_to_source_group(${LIBKINDRV_SUFFIXED_INCLUDE_DIR} "hpp" LIBKINDRV_HPP_FILES_ALL)
append_recursive_files_add_to_source_group(${LIBKINDRV_SUFFIXED_INCLUDE_DIR} "h" LIBKINDRV_HPP_FILES_ALL)

# Add all the source files in all recursive subdirectories and group them accordingly.
append_recursive_files_add_to_source_group(${LIBKINDRV_SUFFIXED_SOURCE_DIR} "cpp" LIBKINDRV_CPP_FILES_ALL)

ADD_LIBRARY(${LIBKINDRV_TARGET_NAME}
	${LIBKINDRV_HPP_FILES_ALL}
	${LIBKINDRV_CPP_FILES_ALL})

TARGET_COMPILE_OPTIONS(${LIBKINDRV_TARGET_NAME} ${COMPILE_OPTIONS})
TARGET_INCLUDE_DIRECTORIES(${LIBKINDRV_TARGET_NAME} PUBLIC ${LIBKINDRV_SUFFIXED_INCLUDE_DIR} PUBLIC ${LIBUSB_1_INCLUDE_DIRS})
TARGET_LINK_LIBRARIES(${LIBKINDRV_TARGET_NAME} PUBLIC ${LIBUSB_1_LIBRARIES})

################################################################################
# rapidjson
################################################################################

# Add all the include files in all recursive subdirectories and group them accordingly.
append_recursive_files_add_to_source_group(${RAPIDJSON_SUFFIXED_INCLUDE_DIR} "hpp" RAPIDJSON_HPP_FILES_ALL)
append_recursive_files_add_to_source_group(${RAPIDJSON_SUFFIXED_INCLUDE_DIR} "h" RAPIDJSON_HPP_FILES_ALL)

# Add all the source files in all recursive subdirectories and group them accordingly.
append_recursive_files_add_to_source_group(${RAPIDJSON_SUFFIXED_SOURCE_DIR} "cpp" RAPIDJSON_CPP_FILES_ALL)

ADD_LIBRARY(${RAPIDJSON_TARGET_NAME}
	${RAPIDJSON_HPP_FILES_ALL}
	${RAPIDJSON_CPP_FILES_ALL})

TARGET_COMPILE_OPTIONS(${RAPIDJSON_TARGET_NAME} ${COMPILE_OPTIONS})
TARGET_INCLUDE_DIRECTORIES(${RAPIDJSON_TARGET_NAME} PUBLIC ${RAPIDJSON_INCLUDE_DIR})
SET_TARGET_PROPERTIES(${RAPIDJSON_TARGET_NAME} PROPERTIES LINKER_LANGUAGE C)

################################################################################
# easylogging
################################################################################

# Add all the include files in all recursive subdirectories and group them accordingly.
append_recursive_files_add_to_source_group(${EASYLOGGING_SUFFIXED_INCLUDE_DIR} "hpp" EASYLOGGING_HPP_FILES_ALL)
append_recursive_files_add_to_source_group(${EASYLOGGING_SUFFIXED_INCLUDE_DIR} "h" EASYLOGGING_HPP_FILES_ALL)

# Add all the source files in all recursive subdirectories and group them accordingly.
append_recursive_files_add_to_source_group(${EASYLOGGING_SUFFIXED_SOURCE_DIR} "cpp" EASYLOGGING_CPP_FILES_ALL)

ADD_LIBRARY(${EASYLOGGING_TARGET_NAME}
	${EASYLOGGING_HPP_FILES_ALL}
	${EASYLOGGING_CPP_FILES_ALL})

TARGET_COMPILE_OPTIONS(${EASYLOGGING_TARGET_NAME} ${COMPILE_OPTIONS})
TARGET_INCLUDE_DIRECTORIES(${EASYLOGGING_TARGET_NAME} PUBLIC ${EASYLOGGING_INCLUDE_DIR})
SET_TARGET_PROPERTIES(${EASYLOGGING_TARGET_NAME} PROPERTIES LINKER_LANGUAGE CPP)

################################################################################
# dirent-win
################################################################################

# Add all the include files in all recursive subdirectories and group them accordingly.
append_recursive_files_add_to_source_group(${DIRENTWIN_SUFFIXED_INCLUDE_DIR} "hpp" DIRENTWIN_HPP_FILES_ALL)
append_recursive_files_add_to_source_group(${DIRENTWIN_SUFFIXED_INCLUDE_DIR} "h" DIRENTWIN_HPP_FILES_ALL)

# Add all the source files in all recursive subdirectories and group them accordingly.
append_recursive_files_add_to_source_group(${DIRENTWIN_SUFFIXED_SOURCE_DIR} "cpp" DIRENTWIN_CPP_FILES_ALL)

ADD_LIBRARY(${DIRENTWIN_TARGET_NAME}
	${DIRENTWIN_HPP_FILES_ALL}
	${DIRENTWIN_CPP_FILES_ALL})

TARGET_COMPILE_OPTIONS(${DIRENTWIN_TARGET_NAME} ${COMPILE_OPTIONS})
TARGET_INCLUDE_DIRECTORIES(${DIRENTWIN_TARGET_NAME} PUBLIC ${DIRENTWIN_INCLUDE_DIR})
SET_TARGET_PROPERTIES(${DIRENTWIN_TARGET_NAME} PROPERTIES LINKER_LANGUAGE C)

################################################################################
# kinjo
################################################################################

# Add all the include files in all recursive subdirectories and group them accordingly.
append_recursive_files_add_to_source_group(${KINJO_SUFFIXED_INCLUDE_DIR} "hpp" KINJO_HPP_FILES_ALL)

# Add all the source files in all recursive subdirectories and group them accordingly.
append_recursive_files_add_to_source_group(${KINJO_SUFFIXED_SOURCE_DIR} "cpp" KINJO_CPP_FILES_ALL)

ADD_EXECUTABLE(${KINJO_TARGET_NAME}
	${KINJO_HPP_FILES_ALL}
	${KINJO_CPP_FILES_ALL})
	
# Under windows, opencv includes <windows.h> which defines min and max as macros.
# To enable their usage as functions we have to prevent this.
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	TARGET_COMPILE_DEFINITIONS(${KINJO_TARGET_NAME} PUBLIC NOMINMAX)
ENDIF()
TARGET_COMPILE_OPTIONS(${KINJO_TARGET_NAME} ${COMPILE_OPTIONS})
TARGET_INCLUDE_DIRECTORIES(${KINJO_TARGET_NAME} PUBLIC ${KINJO_INCLUDE_DIR}
	${EXTERNAL_INCLUDE_DIRS}
	PUBLIC ${OpenCV_INCLUDE_DIR} PUBLIC ${OpenNI_INCLUDE_DIR})
TARGET_LINK_LIBRARIES(${KINJO_TARGET_NAME} PUBLIC ${LIBKINDRV_TARGET_NAME}
	PUBLIC ${OpenCV_LIBS} PUBLIC ${OpenNI_LIBRARY} ${LINK_LIBS})
