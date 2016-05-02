////////////////////////////////////////////////////////////////////////
//
//									IMAGE.CPP
//
//				Main file for the image node library
//
////////////////////////////////////////////////////////////////////////

#define	CCL_OBJ_PREFIX		L"nSpace"
#define	CCL_OBJ_MODULE		L"Image"

// Library implementations
#include "../../lib/imagel/imagel_.h"

// Objects in this module
CCL_OBJLIST_BEGIN()
	// Objects

	// Nodes
	CCL_OBJLIST_ENTRY	(Binary)
	CCL_OBJLIST_ENTRY	(FFT)
	CCL_OBJLIST_ENTRY	(PersistImage)
	CCL_OBJLIST_ENTRY	(Prepare)
	CCL_OBJLIST_ENTRY	(Threshold)

CCL_OBJLIST_END()

