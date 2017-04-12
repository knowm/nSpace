cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("ion")

set	(	
		SOURCES
		ion.cpp
		)

set	(
		HEADERS
		)

add_library (ion SHARED ${SOURCES} ${HEADERS} ../dll.def)

target_link_libraries(ion iol adtl nspcl adtl ccl sysl)

# XML
if (WIN32)
	target_link_libraries(ion msxml2)
endif()
if (UNIX)
	target_link_libraries(ion xml2)
endif()

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER dll)

if (UNIX)
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY 
			LINK_FLAGS "-Wl,--version-script=${PROJECT_SOURCE_DIR}/../../lib/ccl/ccl.version")
endif()

# Installation
install	(
			TARGETS ion 
			RUNTIME DESTINATION bin 
			LIBRARY DESTINATION bin
			ARCHIVE DESTINATION bin
			)

if (MSVC)
	install	(
				FILES				${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}/ion.pdb
				DESTINATION		bin
				CONFIGURATIONS	Debug
				)
endif()
