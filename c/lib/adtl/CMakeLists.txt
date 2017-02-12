cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("adtl")

set (	SOURCES
		avl.cpp
		bool.cpp
		date.cpp
		dict.cpp
		double.cpp
		float.cpp
		int.cpp
		iterate.cpp
		iunknown.cpp
		keys.cpp
		list.cpp
		load.cpp
		long.cpp
		queue.cpp
		remove.cpp
		stack.cpp
		stat.cpp
		store.cpp
		string.cpp
		value.cpp
		variant.cpp
		write.cpp )

set (	HEADERS
		adtl.h
		adtl_.h )

add_library (adtl STATIC ${SOURCES} ${HEADERS})

target_link_libraries(adtl)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER lib)
