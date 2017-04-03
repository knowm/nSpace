cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("sysl")

set (	SOURCES
		cs.cpp
		dbg.cpp
		dl.cpp
		event.cpp
		log.cpp
		memory.cpp
		string.cpp
		thread.cpp
		)

set (	HEADERS
		sysl.h
		sysl_.h )

add_library (sysl STATIC ${SOURCES} ${HEADERS})

target_link_libraries(sysl)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER lib)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE TRUE)

# Installation
install (FILES sysl.h DESTINATION include/lib/sysl)

# Installation
install	(
			TARGETS sysl
			RUNTIME DESTINATION bin 
			LIBRARY DESTINATION lib 
			ARCHIVE DESTINATION lib 
			)

if (MSVC)
	install	(
				FILES				${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}/sysl.pdb
				DESTINATION		lib
				CONFIGURATIONS	Debug
				)
endif()
