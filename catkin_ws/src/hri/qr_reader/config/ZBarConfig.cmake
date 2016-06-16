# - Try to find ZBar
# Once done this will define
#
# ZBar_FOUND        - set to true if ZBar was found
# ZBar_LIBRARY      - link these to use ZBar
# ZBar_INCLUDE_DIR  - path to ZBar header files

if (ZBar_LIBRARY AND ZBar_INCLUDE_DIR)
    # in cache already
	set(ZBar_FOUND TRUE)
else (ZBar_LIBRARY AND ZBar_INCLUDE_DIR)

	set(CANDIDATE_LIB_DIR
		/usr/lib
		/usr/local/lib
		/opt/local/lib
		/sw/lib
		/usr/lib/x86_64-linux-gnu
	)

	set(CANDIDATE_INC_DIR
		/usr/include
		/usr/local/include
	)

	find_path(ZBar_INCLUDE_DIR zbar.h ${CANDIDATE_INC_DIR})
	find_library(ZBar_LIBRARY zbar ${CANDIDATE_LIB_DIR})

	# status output
	include(FindPackageHandleStandardArgs)

	find_package_handle_standard_args(ZBar
		DEFAULT_MSG
		ZBar_LIBRARY
		ZBar_INCLUDE_DIR
	)
	mark_as_advanced(
		ZBar_INCLUDE_DIR
		ZBar_LIBRARY
	)

endif (ZBar_LIBRARY AND ZBar_INCLUDE_DIR)
