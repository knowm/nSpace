
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
add_subdirectory(nspcl)
add_subdirectory(sysl)

if(MSVC)
add_subdirectory(nshxl)
add_subdirectory(plotl)
add_subdirectory(usbl)
add_subdirectory(wmil)
endif()

# External SDKs
if( OpenCV_FOUND )
	add_subdirectory(imagel)
endif()

if( OpenSSL_FOUND )
	add_subdirectory(openssll)
endif()

if ( VTK_FOUND )
	add_subdirectory(visualizel)
endif()
