////////////////////////////////////////////////////////////////////
//
//									nSpace.h
//
//					Main include file for project
//
////////////////////////////////////////////////////////////////////

#pragma once

// Unreal
#include "Engine.h"

// When building Unreal 32-bit, the build system seems to change the
// default packing alignment from the default of 8 bytes. Ensure proper 
// packing in case building inside other environment that do
// not default to the same alignment.
#pragma	pack(push)
#if		defined(_WIN64)
#pragma	pack(16)
#else
#pragma	pack(8)
#endif

// nSpace client
#include "AllowWindowsPlatformTypes.h"
#include <nshxl.h>
#include "HideWindowsPlatformTypes.h"

// Restore packing
#pragma	pack(pop)

