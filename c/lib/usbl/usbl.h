////////////////////////////////////////////////////////////////////////
//
//										USBL.H
//
//									USB library
//
////////////////////////////////////////////////////////////////////////

#ifndef	USBL_H
#define	USBL_H

// System includes
#include "../../lib/nspcl/nspcl.h"

//////////////
// Interfaces
//////////////

///////////
// Classes
///////////

DEFINE_GUID	(	CLSID_Control, 0x2534d0e3, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Device, 0x2534d0b8, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Endpoint, 0x2534d0b9, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

#endif

