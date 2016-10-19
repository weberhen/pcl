# - Try to find Freenect2
# Once done this will define
#
#  FREENECT2_FOUND - system has Freenect2
#  FREENECT2_INCLUDE_DIRS - the Freenect2 include directory
#  FREENECT2_LIBRARY - Link these to use Freenect2
#  FREENECT2_LIBRARIES

find_path(FREENECT2_INCLUDE_DIRS NAMES libfreenect2.hpp
	HINTS
	/usr/local/include/libfreenect2/
	/usr/include/libfreenect2
	/usr/local/include/
	/usr/include/
	}
)
 
find_library(FREENECT2_LIBRARY NAMES freenect2 )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Freenect2
  FOUND_VAR FREENECT2_FOUND
  REQUIRED_VARS FREENECT2_LIBRARY FREENECT2_INCLUDE_DIRS
)
