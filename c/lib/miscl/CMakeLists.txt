cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("miscl")

set (	SOURCES
		asynce.cpp
		asyncq.cpp
		clone.cpp
		compare.cpp
		create.cpp
		dctfmt.cpp
		dctprs.cpp
		debug.cpp
		decode.cpp
		demux.cpp
		dist.cpp
		path.cpp
		strfmt.cpp
		string.cpp
		strprs.cpp
		strstm.cpp
		time.cpp
		timer.cpp
		toggle.cpp
		tokenit.cpp
		type.cpp
		uuid.cpp )

set (	HEADERS
		miscl.h
		miscl_.h )

add_library (miscl STATIC ${SOURCES} ${HEADERS})

target_link_libraries(miscl)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER lib)

