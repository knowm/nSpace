
# Sub-projects
add_subdirectory(adtl)
add_subdirectory(ccl)
add_subdirectory(ext)
add_subdirectory(iol)
add_subdirectory(mathl)
add_subdirectory(medial)
add_subdirectory(miscl)
add_subdirectory(netl)
add_subdirectory(nshl)
add_subdirectory(nshxl)
add_subdirectory(nspcl)
add_subdirectory(plotl)
add_subdirectory(sysl)
add_subdirectory(usbl)

# External SDKs
if( OpenCV_FOUND )
	add_subdirectory(imagel)
endif()

if( OpenSSL_FOUND )
	add_subdirectory(openssll)
endif()
