cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("median")

set	(	
		SOURCES
		median.cpp
		)

set	(
		HEADERS
		)

add_library (median SHARED ${SOURCES} ${HEADERS} ../dll.def)

target_link_libraries(median medial nspcl adtl sysl ccl)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER dll)

if (UNIX)
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY 
			LINK_FLAGS "-Wl,--version-script=${PROJECT_SOURCE_DIR}/../../lib/ccl/ccl.version")
endif()

# Installation
install	(
			TARGETS median 
			RUNTIME DESTINATION bin 
			LIBRARY DESTINATION bin
			ARCHIVE DESTINATION bin
			)

if (MSVC)
	install	(
				FILES				${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}/median.pdb
				DESTINATION		bin
				CONFIGURATIONS	Debug
				)
endif()
