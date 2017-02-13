cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("nshxl")

set	(	
		SOURCES
		client.cpp
#		../nshl/nshl.dir/${CMAKE_BUILD_TYPE}/nshl_i.c
#		../nshl/nshl_i.c
		)

set (	HEADERS
		nshxl.h )

add_library (nshxl STATIC ${SOURCES} ${HEADERS})

target_link_libraries(nshxl)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER lib)

