////////////////////////////////////////////////////////////////////////
//
//									WMIN.CPP
//
//		Main file for the Windows Management Instrumentation library
//
////////////////////////////////////////////////////////////////////////

#define	INITGUID
#define	CCL_OBJ_PREFIX		L"nSpace"
#define	CCL_OBJ_MODULE		L"Wmi"

// Library implementations
#include "../../lib/wmil/wmil_.h"

// Objects in this module
CCL_OBJLIST_BEGIN()
	// Objects

	// Nodes
	CCL_OBJLIST_ENTRY	(Enumerator)
	CCL_OBJLIST_ENTRY	(Instance)
	CCL_OBJLIST_ENTRY	(Locator)
	CCL_OBJLIST_ENTRY	(Service)

CCL_OBJLIST_END()

