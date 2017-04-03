cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("libpng")

set	(
		SOURCES
		png.c
		pngerror.c
		pngget.c
		pngmem.c
		pngpread.c
		pngread.c
		pngrio.c
		pngrtran.c
		pngrutil.c
		pngset.c
		pngtrans.c
		pngwio.c
		pngwrite.c
		pngwtran.c
		pngwutil.c
		)

set	(
		HEADERS
		png.h
		pngconf.h
		)

add_library (libpng STATIC ${SOURCES} ${HEADERS})

target_link_libraries(libpng)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER lib/ext)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE TRUE)

# Installation
install	(
			TARGETS libpng
			RUNTIME DESTINATION bin 
			LIBRARY DESTINATION lib 
			ARCHIVE DESTINATION lib 
			)
