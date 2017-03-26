cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("wmin")

set	(	
		SOURCES
		wmin.cpp
		)

set	(
		HEADERS
		)

add_library (wmin SHARED ${SOURCES} ${HEADERS} ../dll.def)

target_link_libraries(wmin wmil nspcl adtl sysl ccl)

if(WIN32)
	target_link_libraries(wmin wbemuuid.lib)
endif()

include_directories(${wmi_INCLUDE_DIR})

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER dll)

if (UNIX)
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY 
			LINK_FLAGS "-Wl,--version-script=${PROJECT_SOURCE_DIR}/../../lib/ccl/ccl.version")
endif()

# Installation
install	(
			TARGETS wmin 
			RUNTIME DESTINATION bin 
			LIBRARY DESTINATION bin
			ARCHIVE DESTINATION bin
			)

if (MSVC)
	install	(
				FILES				${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}/wmin.pdb
				DESTINATION		bin
				CONFIGURATIONS	Debug
				)
endif()
