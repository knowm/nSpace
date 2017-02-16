cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("medial")

set (	SOURCES
		speak.cpp )

set (	HEADERS
		medial.h
		medial_.h )

add_library (medial STATIC ${SOURCES} ${HEADERS})

target_link_libraries(medial)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER lib)

# Installation
install (FILES medial.h DESTINATION include/lib/medial)

# Installation
install	(
			TARGETS medial
			RUNTIME DESTINATION bin 
			LIBRARY DESTINATION lib 
			ARCHIVE DESTINATION lib 
			)

if (MSVC)
	install	(
				FILES				${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}/medial.pdb
				DESTINATION		lib
				CONFIGURATIONS	Debug
				)
endif()
