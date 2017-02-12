cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("mathn")

set	(	
		SOURCES
		mathn.cpp
		)

set	(
		HEADERS
		)

add_library (mathn SHARED ${SOURCES} ${HEADERS} ../dll.def)

target_link_libraries(mathn mathl nspcl adtl sysl ccl)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER dll)
