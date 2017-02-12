cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("iol")

set (	SOURCES
		bytecach.cpp
		dispatch.cpp
		enumdevs.cpp
		file.cpp
		lock.cpp
		memblck.cpp
		notifydevs.cpp
		persist.cpp
		resource.cpp
		serial.cpp
		stmcopy.cpp
		stmfs.cpp
		stmmem.cpp
		stmprbin.cpp
		stmprxml.cpp
		stmsrc.cpp
		stmsrcfs.cpp
		stmstm.cpp
		stream.cpp )

set (	HEADERS
		iol.h
		iol_.h )

add_library (iol STATIC ${SOURCES} ${HEADERS})

target_link_libraries(iol)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER lib)

