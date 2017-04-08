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
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE TRUE)

# Installation
install (FILES usbl.h DESTINATION include/lib/usbl)

# Installation
install	(
			TARGETS usbl
			RUNTIME DESTINATION bin 
			LIBRARY DESTINATION lib 
			ARCHIVE DESTINATION lib 
			)

if (MSVC)
	install	(
				FILES				${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}/usbl.pdb
				DESTINATION		lib
				CONFIGURATIONS	Debug
				)
endif()
