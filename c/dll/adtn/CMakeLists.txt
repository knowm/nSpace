cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("adtn")

set	(	
		SOURCES
		adtn.cpp
		)

set	(
		HEADERS
		)

add_library (adtn SHARED ${SOURCES} ${HEADERS} ../dll.def)

target_link_libraries(adtn adtl nspcl sysl ccl)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER dll)

# Installation
install	(
			TARGETS adtn 
			RUNTIME DESTINATION bin 
			)

if (MSVC)
	install	(
				FILES				${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}/adtn.pdb
				DESTINATION		bin
				CONFIGURATIONS	Debug
				)
endif()
