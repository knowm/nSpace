cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("nspcl")

set (	SOURCES
		behave.cpp
		connect.cpp
		connlst.cpp
		conns.cpp
		glbnspc.cpp
		keypath.cpp
		link.cpp
		loc.cpp
		namespc.cpp
		nspcl.cpp
		parser.cpp
		reflect.cpp
		tempimpl.cpp
		temploc.cpp
		temporal.cpp
		this.cpp
		value.cpp
		)

set (	HEADERS
		nspcl.h
		nspcl_.h )

add_library (nspcl STATIC ${SOURCES} ${HEADERS})

target_link_libraries(nspcl)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER lib)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE TRUE)

# Installation
install (FILES nspcl.h DESTINATION include/lib/nspcl)

# Installation
install	(
			TARGETS nspcl
			RUNTIME DESTINATION bin 
			LIBRARY DESTINATION lib 
			ARCHIVE DESTINATION lib 
			)

if (MSVC)
	install	(
				FILES				${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}/nspcl.pdb
				DESTINATION		lib
				CONFIGURATIONS	Debug
				)
endif()
