cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("ion")

set	(	
		SOURCES
		ion.cpp
		)

set	(
		HEADERS
		)

add_library (ion SHARED ${SOURCES} ${HEADERS} ../dll.def)

target_link_libraries(ion iol adtl nspcl adtl sysl ccl)

# XML
if (WIN32)
	target_link_libraries(ion msxml2)
endif()

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER dll)

# Installation
install	(
			TARGETS ion 
			RUNTIME DESTINATION bin 
			LIBRARY DESTINATION lib 
			ARCHIVE DESTINATION lib 
			)

if (MSVC)
	install	(
				FILES				${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}/ion.pdb
				DESTINATION		bin
				CONFIGURATIONS	Debug
				)
endif()
