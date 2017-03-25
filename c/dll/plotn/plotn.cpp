////////////////////////////////////////////////////////////////////////
//
//									PLOTN.CPP
//
//			Main file for the plotting node library
//
////////////////////////////////////////////////////////////////////////

#define	CCL_OBJ_PREFIX		L"nSpace"
#define	CCL_OBJ_MODULE		L"Plot"
#include "../../lib/plotl/plotl_.h"

// Objects in this module
CCL_OBJLIST_BEGIN()

	// Objects

	// Nodes
	#ifdef	_WIN32
	CCL_OBJLIST_ENTRY	(Image)
	#endif

CCL_OBJLIST_END()
