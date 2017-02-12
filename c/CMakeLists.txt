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

# Determine if the Visual Studio install supports ATL.
get_filename_component(VC_BIN_DIR ${CMAKE_CXX_COMPILER} DIRECTORY)
get_filename_component(VC_DIR ${VC_BIN_DIR} DIRECTORY)

# Options for external libraries
#    set( MKL_DIR $ENV{MKL_DIR} CACHE FILEPATH "Base directory of Intel MKL Libraries" )
#    set( MKL_INCLUDE_DIR ${MKL_DIR}/include )
#    if( PLATFORM EQUAL 32 )
#        set( MKL_LIB_DIR ${MKL_DIR}/lib/ia32_win )
#        set( MKL_LIBS mkl_sequential.lib mkl_core.lib mkl_intel_c.lib )
#    else()
#        set( MKL_LIB_DIR ${MKL_DIR}/lib/intel64_win )
#        set( MKL_LIBS mkl_sequential.lib mkl_core.lib mkl_intel_lp64.lib )
#    endif()
#    set( AMO_DEPENDENCIES ${CMAKE_CURRENT_SOURCE_DIR}/Dependencies )


# if( PLATFORM EQUAL 32 )
#	 set( OPENCV_LIB_DIR ${OPENCV_DIR}

# Place all DLLs and EXEs in the same directory
# set( CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR}/Debug )
# set( CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR}/Debug )
# set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR}/Debug )
# set( CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR}/Release )
# set( CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR}/Release )
# set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR}/Release )

# Open CV
find_package(OpenCV 3)

# Unicode
add_definitions(-DUNICODE -D_UNICODE)

# Libraries
add_subdirectory("lib")

# DLLs
add_subdirectory("dll")

# Exes
# add_subdirectory("app")

