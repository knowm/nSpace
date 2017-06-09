cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("iol")

set (	SOURCES
		ByteCache.cpp
		Dispatch.cpp
		EnumDevices.cpp
		File.cpp
		Lock.cpp
		MemoryBlock.cpp
		NotifyDevices.cpp
		Persist.cpp
		Resource.cpp
		Serial.cpp
		StreamCopy.cpp
		StmFile.cpp
		StmMemory.cpp
		StmPrsBin.cpp
		StmPrsXML.cpp
		StreamSource.cpp
		StmSrcFile.cpp
		StmSrcStg.cpp
		StmOnByteStm.cpp
		StmStg.cpp
		StmZlib.cpp
		StreamOp.cpp )

set (	HEADERS
		iol.h
		iol_.h )

add_library (iol STATIC ${SOURCES} ${HEADERS})

target_link_libraries(iol)

if (CMAKE_COMPILER_IS_GNUCC)
include_directories(${LIBXML2_INCLUDE_DIR})
endif()

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER lib)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE TRUE)

# Installation
install (FILES iol.h DESTINATION include/lib/iol)

# Installation
install	(
			TARGETS iol
			RUNTIME DESTINATION bin 
			LIBRARY DESTINATION lib 
			ARCHIVE DESTINATION lib 
			)

if (MSVC)
	install	(
				FILES				${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}/iol.pdb
				DESTINATION		lib
				CONFIGURATIONS	Debug
				)
endif()
