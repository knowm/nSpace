cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("wmin")

set	(	
		SOURCES
		wmin.cpp
		)

set	(
		HEADERS
		)

add_library (wmin SHARED ${SOURCES} ${HEADERS} ../dll.def)

target_link_libraries(wmin wmil nspcl adtl sysl ccl)

if(WIN32)
	target_link_libraries(wmin wbemuuid.lib)
endif()

include_directories(${wmi_INCLUDE_DIR})

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER dll)

# Installation
install	(
			TARGETS wmin 
			RUNTIME DESTINATION bin 
			LIBRARY DESTINATION lib 
			ARCHIVE DESTINATION lib 
			)

if (MSVC)
	install	(
				FILES				${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}/wmin.pdb
				DESTINATION		bin
				CONFIGURATIONS	Debug
				)
endif()
