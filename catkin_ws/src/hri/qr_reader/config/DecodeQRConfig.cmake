# - Try to find DecodeQR
# Once done this will define
#
# DecodeQR_FOUND        - set to true if DecodeQR was found
# DecodeQR_LIBRARY      - link these to use DecodeQR
# DecodeQR_INCLUDE_DIR  - path to DecodeQR header files

if (DecodeQR_LIBRARY AND DecodeQR_INCLUDE_DIR)
    # in cache already
	set(DecodeQR_FOUND TRUE)
else (DecodeQR_LIBRARY AND DecodeQR_INCLUDE_DIR)

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

	find_path(DecodeQR_INCLUDE_DIR decodeqr.h ${CANDIDATE_INC_DIR})
	find_library(DecodeQR_LIBRARY decodeqr ${CANDIDATE_LIB_DIR})

	# status output
	include(FindPackageHandleStandardArgs)

	find_package_handle_standard_args(DecodeQR
		DEFAULT_MSG
		DecodeQR_LIBRARY
		DecodeQR_INCLUDE_DIR
	)
	mark_as_advanced(
		DecodeQR_INCLUDE_DIR
		DecodeQR_LIBRARY
	)

endif (DecodeQR_LIBRARY AND DecodeQR_INCLUDE_DIR)
