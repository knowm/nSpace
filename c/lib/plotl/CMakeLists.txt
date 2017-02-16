cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("plotl")

set (	SOURCES
		gnuplotsrvr.cpp
		image.cpp
		)

set (	HEADERS
		plotl.h
		plotl_.h )

add_library (plotl STATIC ${SOURCES} ${HEADERS})

target_link_libraries(plotl)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER lib)

# Installation
install (FILES plotl.h DESTINATION include/lib/plotl)

# Installation
install	(
			TARGETS plotl
			RUNTIME DESTINATION bin 
			LIBRARY DESTINATION lib 
			ARCHIVE DESTINATION lib 
			)

if (MSVC)
	install	(
				FILES				${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}/plotl.pdb
				DESTINATION		lib
				CONFIGURATIONS	Debug
				)
endif()
