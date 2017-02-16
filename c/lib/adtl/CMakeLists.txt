cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("adtl")

set (	SOURCES
		AVL.cpp
		Bool.cpp
		Date.cpp
		Dictionary.cpp
		Double.cpp
		Float.cpp
		Int.cpp
		Iterate.cpp
		IUnknown.cpp
		Keys.cpp
		List.cpp
		Load.cpp
		Long.cpp
		Queue.cpp
		Remove.cpp
		Stack.cpp
		Stat.cpp
		Store.cpp
		String.cpp
		Value.cpp
		Variant.cpp
		Write.cpp )

set (	HEADERS
		adtl.h
		adtl_.h )

add_library (adtl STATIC ${SOURCES} ${HEADERS})

target_link_libraries(adtl)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER lib)

# Installation
install (FILES adtl.h DESTINATION include/lib/adtl)

# Installation
install	(
			TARGETS adtl
			RUNTIME DESTINATION bin 
			LIBRARY DESTINATION lib 
			ARCHIVE DESTINATION lib 
			)

if (MSVC)
	install	(
				FILES				${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}/adtl.pdb
				DESTINATION		lib
				CONFIGURATIONS	Debug
				)
endif()
