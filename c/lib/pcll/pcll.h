////////////////////////////////////////////////////////////////////////
//
//										PCLL.H
//
//				Point Cloud Library (PCL) node library
//
////////////////////////////////////////////////////////////////////////

#ifndef	PCLL_H
#define	PCLL_H

// System includes
#define	NOMINMAX											// Disable unused legacy macros
#include "../../lib/nspcl/nspcl.h"

//////////////
// Interfaces
//////////////

///////////
// Classes
///////////

DEFINE_GUID	(	CLSID_Create, 0x2534d0df, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_ImageToCloud, 0x2534d0de, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

#endif

