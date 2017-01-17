////////////////////////////////////////////////////////////////////////
//
//									PCLN.CPP
//
//				Main file for the Point Cloud Library node library
//
////////////////////////////////////////////////////////////////////////

#define	CCL_OBJ_PREFIX		L"nSpace"
#define	CCL_OBJ_MODULE		L"Pcl"

// Library implementations
#include "../../lib/pcll/pcll.h"
#include "../../lib/pcll/pcll_.h"

// Objects in this module
CCL_OBJLIST_BEGIN()
	// Objects

	// Nodes
	CCL_OBJLIST_ENTRY	(Create)
	CCL_OBJLIST_ENTRY	(ImageToCloud)
	CCL_OBJLIST_ENTRY	(Normal)

CCL_OBJLIST_END()

