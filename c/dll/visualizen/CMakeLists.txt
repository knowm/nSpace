cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("visualizen")

set	(	
		SOURCES
		visualizen.cpp
		)

set	(
		HEADERS
		)

add_library (visualizen SHARED ${SOURCES} ${HEADERS} ../dll.def)

target_link_libraries(visualizen visualizel adtl nspcl sysl ccl)

# VTK
include_directories(${VTK_INCLUDE_DIRS})
target_link_libraries(visualizen ${VTK_LIBRARIES})

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER dll)

if (UNIX)
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY 
			LINK_FLAGS "-Wl,--version-script=${PROJECT_SOURCE_DIR}/../../lib/ccl/ccl.version")
endif()

# Installation
install	(
			TARGETS visualizen 
			RUNTIME DESTINATION bin 
			LIBRARY DESTINATION bin
			ARCHIVE DESTINATION bin
			)

if (MSVC)
	install	(
				FILES				${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}/visualizen.pdb
				DESTINATION		bin
				CONFIGURATIONS	Debug
				)
endif()
