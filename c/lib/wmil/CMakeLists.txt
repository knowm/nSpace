cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("wmil")

set	(
		SOURCES
		Enumerator.cpp
		Instance.cpp
		Locator.cpp
		Service.cpp
		wmil.cpp
		)

set (	HEADERS
		wmil.h
		wmil_.h )

add_library (wmil STATIC ${SOURCES} ${HEADERS})

target_link_libraries(wmil)

include_directories(${OPENSSL_INCLUDE_DIR})

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER lib)

# Installation
install (FILES wmil.h DESTINATION include/lib/wmil)

# Installation
install	(
			TARGETS wmil
			RUNTIME DESTINATION bin 
			LIBRARY DESTINATION lib 
			ARCHIVE DESTINATION lib 
			)

if (MSVC)
	install	(
				FILES				${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}/wmil.pdb
				DESTINATION		lib
				CONFIGURATIONS	Debug
				)
endif()
