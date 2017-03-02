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

target_link_libraries(sysn sysl ccl)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER dll)

# Installation
install	(
			TARGETS sysn 
			RUNTIME DESTINATION bin 
			LIBRARY DESTINATION lib 
			ARCHIVE DESTINATION lib 
			)

if (MSVC)
	install	(
				FILES				${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}/sysn.pdb
				DESTINATION		bin
				CONFIGURATIONS	Debug
				)
endif()