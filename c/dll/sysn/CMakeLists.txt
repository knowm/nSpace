cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("sysn")

set	(	
		SOURCES
		sysn.cpp
		)

set	(
		HEADERS
		)

add_library (sysn SHARED ${SOURCES} ${HEADERS} ../dll.def)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER dll)

target_link_libraries(sysn ccl sysl)

if (UNIX)
target_link_libraries(sysn pthread)
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY 
			LINK_FLAGS "-Wl,--version-script=${PROJECT_SOURCE_DIR}/../../lib/ccl/ccl.version")
endif()

# Installation
install	(
			TARGETS sysn 
			RUNTIME DESTINATION bin 
			LIBRARY DESTINATION bin
			ARCHIVE DESTINATION bin
			)

if (MSVC)
	install	(
				FILES				${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}/sysn.pdb
				DESTINATION		bin
				CONFIGURATIONS	Debug
				)
endif()
