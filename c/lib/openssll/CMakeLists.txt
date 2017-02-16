cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("openssll")

set	(
		SOURCES
		connect.cpp
		openssll.cpp
		pem.cpp
		rsa.cpp
		sign.cpp
		verify.cpp
		)

set (	HEADERS
		openssll.h
		openssll_.h )

add_library (openssll STATIC ${SOURCES} ${HEADERS})

target_link_libraries(openssll)

include_directories(${OPENSSL_INCLUDE_DIR})
# MESSAGE(STATUS ${OPENSSL_INCLUDE_DIR})

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER lib)

# Installation
install (FILES openssll.h DESTINATION include/lib/openssll)

# Installation
install	(
			TARGETS openssll
			RUNTIME DESTINATION bin 
			LIBRARY DESTINATION lib 
			ARCHIVE DESTINATION lib 
			)

if (MSVC)
	install	(
				FILES				${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}/openssll.pdb
				DESTINATION		lib
				CONFIGURATIONS	Debug
				)
endif()
