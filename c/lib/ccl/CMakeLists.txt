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

if(UNIX)
add_library (ccl_impl STATIC ccl_impl.cpp)
endif()

target_link_libraries(ccl)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER lib)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE TRUE)

# Installation
install (FILES ccl.h DESTINATION include/lib/ccl)

# Installation
install	(
			TARGETS ccl
			RUNTIME DESTINATION bin 
			LIBRARY DESTINATION lib 
			ARCHIVE DESTINATION lib 
			)

if (MSVC)
	install	(
				FILES				${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}/ccl.pdb
				DESTINATION		lib
				CONFIGURATIONS	Debug
				)
endif()
