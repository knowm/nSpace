cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("mathl")

set (	SOURCES
		binary.cpp
		counter.cpp
		datablk.cpp
		formula.cpp
		function.cpp
		line.cpp
		mat3d.cpp
		mathl.cpp
		trans3d.cpp
		unary.cpp )

set (	HEADERS
		mathl.h
		mathl_.h )

add_library (mathl STATIC ${SOURCES} ${HEADERS})

target_link_libraries(mathl)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER lib)

# Installation
install (FILES mathl.h DESTINATION include/lib/mathl)
