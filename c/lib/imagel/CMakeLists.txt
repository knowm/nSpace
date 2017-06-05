cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("imagel")

set	(
		SOURCES
		At.cpp
		Binary.cpp
		CascadeClassifier.cpp
		Codec.cpp
		Contours.cpp
		Convert.cpp
		Create.cpp
		cvMatRef.cpp
		Distance.cpp
		Draw.cpp
		FaceRecognizer.cpp
		Features.cpp
		FFT.cpp
		Flip.cpp
		Gradient.cpp
		libJpeg.cpp
		libPng.cpp
		Match.cpp
		Morph.cpp
		Normalize.cpp
		opencv.cpp
		PersistImage.cpp
		Prepare.cpp
		Resize.cpp
		Roi.cpp
		Smooth.cpp
		Stats.cpp
		Threshold.cpp
		Tomography.cpp
		Transform.cpp
		VideoCapture.cpp
		VideoWriter.cpp
		)

set (	HEADERS
		imagel.h
		imagel_.h )

add_library (imagel STATIC ${SOURCES} ${HEADERS})

target_link_libraries(imagel)

include_directories(${OpenCV_INCLUDE_DIRS})

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER lib)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE TRUE)

# OpenCV contrib build detection
# Get the first OpenCV include directory and look for a 'contrib' header
list (GET OpenCV_INCLUDE_DIRS 0 OpenCV_INCLUDE_DIR_FIRST)
if(EXISTS ${OpenCV_INCLUDE_DIR_FIRST}/opencv2/face/facerec.hpp)
	add_definitions(-DHAVE_OPENCV_CONTRIB)
endif()

# CUDA
list (GET OpenCV_INCLUDE_DIRS 0 OpenCV_INCLUDE_DIR_FIRST)
if(EXISTS ${OpenCV_INCLUDE_DIR_FIRST}/opencv2/cudafeatures2d.hpp)
	add_definitions(-DHAVE_OPENCV_CUDA)
endif()

# Installation
install (FILES imagel.h DESTINATION include/lib/imagel)

# Installation
install	(
			TARGETS imagel
			RUNTIME DESTINATION bin 
			LIBRARY DESTINATION lib 
			ARCHIVE DESTINATION lib 
			)

if (MSVC)
	install	(
				FILES				${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}/imagel.pdb
				DESTINATION		lib
				CONFIGURATIONS	Debug
				)
endif()
