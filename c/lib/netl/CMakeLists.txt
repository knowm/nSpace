cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("netl")

set (	SOURCES
		address.cpp
		avail.cpp
		client.cpp
		dgram.cpp
		intfs.cpp
		multicast.cpp
		netl.cpp
		persist.cpp
		recv.cpp
		send.cpp
		socket.cpp )

set (	HEADERS
		netl.h
		netl_.h )

add_library (netl STATIC ${SOURCES} ${HEADERS})

target_link_libraries(netl)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER lib)

# Installation
install (FILES netl.h DESTINATION include/lib/netl)

# Installation
install	(
			TARGETS netl
			RUNTIME DESTINATION bin 
			LIBRARY DESTINATION lib 
			ARCHIVE DESTINATION lib 
			)

if (MSVC)
	install	(
				FILES				${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}/netl.pdb
				DESTINATION		lib
				CONFIGURATIONS	Debug
				)
endif()
