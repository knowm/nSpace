cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("sqll")

set	(
		SOURCES
		Connection.cpp
		Handle.cpp
		)

set (	HEADERS
		sqll.h
		sqll_.h )

add_library (sqll STATIC ${SOURCES} ${HEADERS})

target_link_libraries(sqll)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER lib)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE TRUE)

# Installation
install (FILES sqll.h DESTINATION include/lib/sqll)

# Installation
install	(
			TARGETS sqll
			RUNTIME DESTINATION bin 
			LIBRARY DESTINATION lib 
			ARCHIVE DESTINATION lib 
			)

if (MSVC)
	install	(
				FILES				${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}/sqll.pdb
				DESTINATION		lib
				CONFIGURATIONS	Debug
				)
endif()
