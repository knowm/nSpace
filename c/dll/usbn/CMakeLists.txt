cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("usbn")

set	(	
		SOURCES
		usbn.cpp
		)

set	(
		HEADERS
		)

add_library (usbn SHARED ${SOURCES} ${HEADERS} ../dll.def)

target_link_libraries(usbn usbl adtl nspcl sysl ccl)

# WinUSB
if (WIN32)
	target_link_libraries(usbn winusb)
endif()

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER dll)

# Installation
install	(
			TARGETS usbn 
			RUNTIME DESTINATION bin 
			LIBRARY DESTINATION bin
			ARCHIVE DESTINATION bin
			)

if (MSVC)
	install	(
				FILES				${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}/usbn.pdb
				DESTINATION		bin
				CONFIGURATIONS	Debug
				)
endif()
