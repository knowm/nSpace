cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("nshxl")

set	(	
		SOURCES
		client.cpp
		)

set (	HEADERS
		nshxl.h )

add_library (nshxl STATIC ${SOURCES} ${HEADERS})

target_link_libraries(nshxl)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER lib)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE TRUE)

# Installation
install (FILES nshxl.h DESTINATION include/lib/nshxl)
install	(
			TARGETS nshxl
			RUNTIME DESTINATION bin 
			LIBRARY DESTINATION lib 
			ARCHIVE DESTINATION lib 
			)

if (MSVC)
	install	(
				FILES				${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}/nshxl.pdb
				DESTINATION		lib
				CONFIGURATIONS	Debug
				)
endif()
