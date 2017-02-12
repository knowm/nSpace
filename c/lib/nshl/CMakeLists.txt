cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("nshl")

set (	SOURCES
		maindef.cpp
		mainsvc.cpp
		mainx.cpp
		nshl.cpp
		recepx.cpp
		shell.cpp
		shellx.cpp
		shfctx.cpp
		spcx.cpp
		nshl.idl
		)

set (	HEADERS
		nshl.h
		nshl_.h )

add_library (nshl STATIC ${SOURCES} ${HEADERS})

target_link_libraries(nshl)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER lib)

