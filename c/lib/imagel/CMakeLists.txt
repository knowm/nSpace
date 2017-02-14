cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("imagel")

set	(
		SOURCES
		at.cpp
		binary.cpp
		codec.cpp
		contours.cpp
		convert.cpp
		create.cpp
		cvmatref.cpp
		distance.cpp
		draw.cpp
		features.cpp
		fft.cpp
		flip.cpp
		gradient.cpp
		jpeg.cpp
		match.cpp
		morph.cpp
		normal.cpp
		opencv.cpp
		persist.cpp
		png.cpp
		prepare.cpp
		resize.cpp
		roi.cpp
		smooth.cpp
		stats.cpp
		thresh.cpp
		)

set (	HEADERS
		imagel.h
		imagel_.h )

add_library (imagel STATIC ${SOURCES} ${HEADERS})

target_link_libraries(imagel)

include_directories(${OpenCV_INCLUDE_DIRS})

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER lib)

# Installation
install (FILES imagel.h DESTINATION include/lib/imagel)
