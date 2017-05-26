cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("visualizel")

set	(
		SOURCES
		Points.cpp
		Renderable.cpp
		Render.cpp
		)

set (	HEADERS
		visualizel.h
		visualizel_.h )

add_library (visualizel STATIC ${SOURCES} ${HEADERS})

target_link_libraries(visualizel)

include_directories(${VTK_INCLUDE_DIRS})

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER lib)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE TRUE)

# Installation
install (FILES visualizel.h DESTINATION include/lib/visualizel)

# Installation
install	(
			TARGETS visualizel
			RUNTIME DESTINATION bin 
			LIBRARY DESTINATION lib 
			ARCHIVE DESTINATION lib 
			)

if (MSVC)
	install	(
				FILES				${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}/visualizel.pdb
				DESTINATION		lib
				CONFIGURATIONS	Debug
				)
endif()
