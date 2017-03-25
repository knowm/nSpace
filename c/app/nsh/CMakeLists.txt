cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("nsh")

set	(	
		SOURCES
		main.cpp
		)

set	(
		HEADERS
		nsh.h
		)

add_executable (nsh WIN32 ${SOURCES} ${HEADERS})


target_link_libraries(nsh adtl nshl nspcl sysl ccl)

if (WIN32)
	target_link_libraries(nsh nshxl comctl32)
endif()

if (UNIX)
	target_link_libraries(nsh dl)
endif()

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER app)

# Installation
install (TARGETS nsh DESTINATION bin)
