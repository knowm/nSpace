cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("ccl")

set	(	
		SOURCES
		ccl.cpp
		factory.cpp
		object.cpp
		register.cpp
		)

add_library (ccl STATIC ${SOURCES} ${HEADERS})

target_link_libraries(ccl)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER lib)
