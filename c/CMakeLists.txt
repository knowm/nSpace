cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Configs" FORCE)

# Installation.  The default if not specified is "C:\Program files\nSpace"
set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Installation Directory")

project("nSpace")

# Use solution folders
set_property(GLOBAL PROPERTY USE_FOLDERS ON)


# Determine platform (32 or 64-bit)
if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    set( PLATFORM 64 )
    set( PLATFORM_NAME "x64" )
else()
    set( PLATFORM 32 )
    set( PLATFORM_NAME "x86" )
endif()

get_filename_component(VC_BIN_DIR ${CMAKE_CXX_COMPILER} DIRECTORY)
get_filename_component(VC_DIR ${VC_BIN_DIR} DIRECTORY)

# Place all DLLs and EXEs in the same directory
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR}/Debug )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR}/Debug )
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR}/Debug )
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR}/Release )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR}/Release )
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR}/Release )

# SDKs
find_package(OpenCV)
find_package(OpenSSL)

# NOTE: Must be the <install>/lib/cmake/vtk-<version> directory not just the install dir.
# NOTE NOTE NOTE.  VTK seems to rely a lot on static intialization.  This may be why
# linking to VTK DLLs from COM DLLs is not a good idea (load order, etc).  For now
# VTK is statically link to avoid crashes on exit.  Revsisit this in the future,
# is there some sort of 'uninitialization' call that can manually be made before the DLL is unloaded ?
find_package(VTK)

if (UNIX)
find_package(LibXml2)
find_package(ZLIB)
endif()

# Unicode
add_definitions(-DUNICODE -D_UNICODE)

# Libraries
add_subdirectory("lib")

# DLLs
add_subdirectory("dll")

# Exes
add_subdirectory("app")

# Install the graphs into the destination
install	(
			DIRECTORY	"${PROJECT_SOURCE_DIR}/../graph" 
			DESTINATION "."
			PATTERN		"*.nspc"
			)

# Add graphs to visual studio project NOT working yet
# NOTE: Including the 'relative' tag makes cmake crash
#file(GLOB_RECURSE GRAPH_SRCS RELATIVE "../graph/*.nspc")
#MESSAGE(STATUS "Files:" )
#MESSAGE(STATUS "${GRAPH_SRCS}" )

# MESSAGE(STATUS ${PROJECT_SOURCE_DIR} )
# MESSAGE(STATUS "${PROJECT_SOURCE_DIR}/../graph" )
# file(GLOB_RECURSE GRAPH_SRCS ${PROJECT_SOURCE_DIR}/../graph *.nspc)
# MESSAGE(STATUS "${GRAPH_SRCS}" )
# set(GRAPH_SRCS ${PROJECT_SOURCE_DIR}/../graph/app/auto.nspc)
# MESSAGE(STATUS "${GRAPH_SRCS}" )
# MESSAGE(STATUS "Hi!!!")
# SOURCE_GROUP(graph ${GRAPH_SRCS})
# SOURCE_GROUP("${PROJECT_SOURCE_DIR}/../graph" REGULAR_EXPRESSION ${GRAPH_SRCS})
#file(GLOB_RECURSE GRAPH_SRCS "G:\\dev\\nSpace\\graph" *.nspc)
#MESSAGE(STATUS "${GRAPH_SRCS}" )
#file(GLOB_RECURSE GRAPH_SRCS ${PROJECT_SOURCE_DIR}/../graph *.nspc)
#add_custom_target ( graph SOURCES ${GRAPH_SRCS} )
#SOURCE_GROUP(graph FILES ${PROJECT_SOURCE_DIR}/../../../graph/app/auto.nspc)

# Display all CMAKE variables
#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()
