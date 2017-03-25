////////////////////////////////////////////////////////////////////////
//
//									MEDIAN.CPP
//
//				Main file for the media node library
//
////////////////////////////////////////////////////////////////////////

#define	CCL_OBJ_PREFIX		L"nSpace"
#define	CCL_OBJ_MODULE		L"Media"
#include "../../lib/medial/medial_.h"

// Objects in this module
CCL_OBJLIST_BEGIN()

	// Nodes
	#ifdef	_WIN32
	CCL_OBJLIST_ENTRY	(Speak)
	#endif

CCL_OBJLIST_END()
