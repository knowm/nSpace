cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Configs" FORCE)

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
find_package(OpenCV 3)
find_package(OpenSSL)

# Unicode
add_definitions(-DUNICODE -D_UNICODE)

# Libraries
add_subdirectory("lib")

# DLLs
add_subdirectory("dll")

# Exes
add_subdirectory("app")
