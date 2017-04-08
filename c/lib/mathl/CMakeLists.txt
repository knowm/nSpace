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
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE TRUE)

# Installation
install (FILES mathl.h DESTINATION include/lib/mathl)

# Installation
install	(
			TARGETS mathl
			RUNTIME DESTINATION bin 
			LIBRARY DESTINATION lib 
			ARCHIVE DESTINATION lib 
			)

if (MSVC)
	install	(
				FILES				${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}/mathl.pdb
				DESTINATION		lib
				CONFIGURATIONS	Debug
				)
endif()
