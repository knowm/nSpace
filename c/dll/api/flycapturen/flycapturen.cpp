////////////////////////////////////////////////////////////////////////
//
//									FLYCAPTUREN.CPP
//
//				Main file for the Point Grey FlyCapture SDK library
//
////////////////////////////////////////////////////////////////////////

#define	INITGUID
#define	CCL_OBJ_PREFIX		L"nSpace"
#define	CCL_OBJ_MODULE		L"FlyCapture"

// Library implementations
#include "../../../lib/api/flycapturel/flycapturel_.h"

// Objects in this module
CCL_OBJLIST_BEGIN()
	// Objects

	// Nodes
	CCL_OBJLIST_ENTRY	(Camera)
	CCL_OBJLIST_ENTRY	(Enum)

CCL_OBJLIST_END()

