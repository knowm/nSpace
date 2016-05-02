////////////////////////////////////////////////////////////////////////
//
//										IMAGEL.H
//
//							Image processing library
//
////////////////////////////////////////////////////////////////////////

#ifndef	IMAGEL_H
#define	IMAGEL_H

// System includes
#include "../../lib/nspcl/nspcl.h"

//////////////
// Interfaces
//////////////

///////////
// Classes
///////////

DEFINE_GUID	(	CLSID_Binary, 0x2534d017, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_FFT, 0x2534d011, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_PersistImage, 0x2534d012, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Prepare, 0x2534d013, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Threshold, 0x2534d014, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

#endif

