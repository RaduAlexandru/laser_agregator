# - Try to find the LIBIGL library
# Once done this will define
#
#  LIBIGL_FOUND - system has LIBIGL
#  LIBIGL_INCLUDE_DIR - **the** LIBIGL include directory
#  LIBIGL_INCLUDE_DIRS - LIBIGL include directories
#  LIBIGL_SOURCES - the LIBIGL source files

if(NOT LIBIGL_FOUND)
	find_path(LIBIGL_INCLUDE_DIR
		NAMES igl/readOBJ.h
	   	PATHS ${PROJECT_SOURCE_DIR}/deps/libigl/include
		DOC "The libigl include directory"
		NO_DEFAULT_PATH)

	if(LIBIGL_INCLUDE_DIR)
	   set(LIBIGL_FOUND TRUE)
	   set(LIBIGL_INCLUDE_DIRS ${LIBIGL_INCLUDE_DIR})
	else()
	   message("+-------------------------------------------------+")
	   message("| libigl not found, please run download_libigl.sh |")
	   message("+-------------------------------------------------+")
	   message(FATAL_ERROR "")
	endif()
endif()
