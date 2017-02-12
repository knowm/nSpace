# Solution folder
# set_target_properties(${PROJECT_NAME} PROPERTIES USE_FOLDERS Test)

add_subdirectory(adtl)
add_subdirectory(ccl)
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

if( OPENCV_FOUND )
add_subdirectory(imagel)
endif()

# add_subdirectory(openssll)
