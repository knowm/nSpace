cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("zlib")

set	(
		SOURCES
		adler32.c
		compress.c
		crc32.c
		deflate.c
		example.c
		gzio.c
		infback.c
		inffast.c
		inflate.c
		inftrees.c
		trees.c
		uncompr.c
		zutil.c
		)

set	(
		HEADERS
		crc32.h
		deflate.h
		inffast.h
		inffixed.h
		inflate.h
		inftrees.h
		trees.h
		zconf.h
		zconf.in.h
		zlib.h
		zutil.h
		)

add_library (zlib STATIC ${SOURCES} ${HEADERS})

target_link_libraries(zlib)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER lib/ext)

# Installation
install	(
			TARGETS zlib
			RUNTIME DESTINATION bin 
			LIBRARY DESTINATION lib 
			ARCHIVE DESTINATION lib 
			)
