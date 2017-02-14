cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("miscn")

set	(	
		SOURCES
		miscn.cpp
		)

set	(
		HEADERS
		)

add_library (miscn SHARED ${SOURCES} ${HEADERS} ../dll.def)

target_link_libraries(miscn miscl nspcl adtl sysl ccl)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER dll)

# Installation
install	(
			TARGETS miscn 
			RUNTIME DESTINATION bin 
			LIBRARY DESTINATION lib 
			ARCHIVE DESTINATION lib 
			)

if (MSVC)
	install	(
				FILES				${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}/miscn.pdb
				DESTINATION		bin
				CONFIGURATIONS	Debug
				)
endif()
