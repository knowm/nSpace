cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("plotn")

set	(	
		SOURCES
		plotn.cpp
		)

set	(
		HEADERS
		)

add_library (plotn SHARED ${SOURCES} ${HEADERS} ../dll.def)

target_link_libraries(plotn plotl adtl nspcl sysl ccl)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER dll)
