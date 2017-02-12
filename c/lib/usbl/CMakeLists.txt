cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("usbl")

set	(
		SOURCES
		device.cpp
		endp.cpp
		)

set	(	
		HEADERS
		usbl.h
		usbl_.h
		)

add_library (usbl STATIC ${SOURCES} ${HEADERS})

target_link_libraries(usbl)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER lib)

