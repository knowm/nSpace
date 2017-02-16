cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project("nshl")

set (	SOURCES
		maindef.cpp
		mainsvc.cpp
		mainx.cpp
		nshl.cpp
		recepx.cpp
		shell.cpp
		shellx.cpp
		shfctx.cpp
		spcx.cpp
		nshl.idl
		)

set (	HEADERS
		nshl.h
		nshl_.h )

add_library (nshl STATIC ${SOURCES} ${HEADERS})

target_link_libraries(nshl)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER lib)

# Copy the TLB into the build directory post-build since during development
# this directory is used/registered, not the install.  Copy from both directories
# to avoid having to worry about debug/release targets
# ADD_CUSTOM_COMMAND (
# 	TARGET TLB
# 	POST_BUILD
# 	COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG}/nshl.dir/Debug/nshl.tlb ${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG}/.
# 	COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE}/nshl.dir/Release/nshl.tlb ${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG}/.
# 	)

# Installation
install (FILES nshl.h DESTINATION include/lib/nshl)
install	(
			TARGETS nshl
			LIBRARY DESTINATION lib 
			ARCHIVE DESTINATION lib 
			)

if (MSVC)
	install	(
				FILES				${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}/nshl.pdb
				DESTINATION		lib
				CONFIGURATIONS	Debug
				)

	install	(
				FILES				${CMAKE_CURRENT_BINARY_DIR}/nshl.dir/Debug/nshl.tlb
				DESTINATION		bin
				CONFIGURATIONS	Debug
				)
	install	(
				FILES				${CMAKE_CURRENT_BINARY_DIR}/nshl.dir/Release/nshl.tlb
				DESTINATION		bin
				CONFIGURATIONS	Release
				)

endif()
