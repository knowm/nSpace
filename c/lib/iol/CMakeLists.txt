cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("iol")

set (	SOURCES
		ByteCachE.cpp
		Dispatch.cpp
		EnumDevices.cpp
		File.cpp
		Lock.cpp
		MemoryBlock.cpp
		NotifyDevs.cpp
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
		StmOnByteStm.cpp
		StreamOp.cpp )

set (	HEADERS
		iol.h
		iol_.h )

add_library (iol STATIC ${SOURCES} ${HEADERS})

target_link_libraries(iol)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER lib)

