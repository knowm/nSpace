////////////////////////////////////////////////////////////////////////
//
//										NSHL.H
//
//								nSpace shell library
//
////////////////////////////////////////////////////////////////////////

#ifndef	NSHL_H
#define	NSHL_H

// System includes
#include "../adtl/adtl.h"

//
// GUID
//

// Defined here again to avoid having to include output from MIDL compiler.

DEFINE_GUID	(	IID_IListenX, 0x2534d084, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	IID_INamespaceX, 0x2534d085, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	IID_IShellX, 0x2534d086, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

#endif
