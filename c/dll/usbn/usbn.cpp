////////////////////////////////////////////////////////////////////////
//
//									USBN.CPP
//
//				Main file for the USB interface library
//
////////////////////////////////////////////////////////////////////////

#define	INITGUID
#define	CCL_OBJ_PREFIX		L"nSpace"
#define	CCL_OBJ_MODULE		L"Usb"

// Library implementations
#include "../../lib/usbl/usbl_.h"

// Objects in this module
CCL_OBJLIST_BEGIN()
	// Objects

	// Nodes
	CCL_OBJLIST_ENTRY	(Device)
	CCL_OBJLIST_ENTRY	(Endpoint)

CCL_OBJLIST_END()

