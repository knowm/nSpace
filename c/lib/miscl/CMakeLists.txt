cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("miscl")

set (	SOURCES
		AsyncEmit.cpp
		AsyncQ.cpp
		Clone.cpp
		Compare.cpp
		Create.cpp
		DictFormat.cpp
		DictParse.cpp
		Debug.cpp
		Decode.cpp
		Demux.cpp
		Dist.cpp
		Path.cpp
		StringFormat.cpp
		StringOp.cpp
		StringParse.cpp
		StringStream.cpp
		TimeOp.cpp
		Timer.cpp
		Toggle.cpp
		TokenIt.cpp
		Type.cpp
		UUIDOp.cpp )

set (	HEADERS
		miscl.h
		miscl_.h )

add_library (miscl STATIC ${SOURCES} ${HEADERS})

target_link_libraries(miscl)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER lib)

