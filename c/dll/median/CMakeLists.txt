cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("median")

set	(	
		SOURCES
		median.cpp
		)

set	(
		HEADERS
		)

add_library (median SHARED ${SOURCES} ${HEADERS} ../dll.def)

target_link_libraries(median medial nspcl adtl sysl ccl)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER dll)
