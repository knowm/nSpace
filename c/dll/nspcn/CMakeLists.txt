cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("nspcn")

set	(	
		SOURCES
		nspcn.cpp
		)

set	(
		HEADERS
		)

add_library (nspcn SHARED ${SOURCES} ${HEADERS} ../dll.def)

target_link_libraries(nspcn nspcl adtl sysl ccl)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER dll)

# Installation
install	(
			TARGETS nspcn 
			RUNTIME DESTINATION bin 
			LIBRARY DESTINATION lib 
			ARCHIVE DESTINATION lib 
			)

if (MSVC)
	install	(
				FILES				${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}/nspcn.pdb
				DESTINATION		bin
				CONFIGURATIONS	Debug
				)
endif()
