cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("netn")

set	(	
		SOURCES
		netn.cpp
		)

set	(
		HEADERS
		)
add_library (netn SHARED ${SOURCES} ${HEADERS} ../dll.def)

target_link_libraries(netn netl nspcl adtl sysl ccl)


# WinSock
if (WIN32)
	target_link_libraries(netn ws2_32 iphlpapi)
endif()

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER dll)
