cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("imagen")

set	(	
		SOURCES
		imagen.cpp
		)

set	(
		HEADERS
		)
add_library (imagen SHARED ${SOURCES} ${HEADERS} ../dll.def)

target_link_libraries(imagen imagel mathl nspcl adtl sysl ccl)
target_link_libraries(imagen libjpeg libpng zlib)
target_link_libraries(imagen ${OpenCV_LIBS})

include_directories(${OpenCV_INCLUDE_DIRS})

# OpenCV
# if (WIN32)
# 	target_link_libraries(imagen ws2_32 iphlpapi)
# endif()

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER dll)

# Installation
install	(
			TARGETS imagen 
			RUNTIME DESTINATION bin 
			LIBRARY DESTINATION lib 
			ARCHIVE DESTINATION lib 
			)

if (MSVC)
	install	(
				FILES				${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}/imagen.pdb
				DESTINATION		bin
				CONFIGURATIONS	Debug
				)
endif()
