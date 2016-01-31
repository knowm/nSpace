////////////////////////////////////////////////////////////////////////
//
//									ADTN.CPP
//
//					Main file for the ADTN library
//
////////////////////////////////////////////////////////////////////////

#define	CCL_OBJ_PREFIX		L"nSpace"
#define	CCL_OBJ_MODULE		L"Adt"

// Library implementations
#include "../../lib/adtl/adtl_.h"

// Objects in this module
CCL_OBJLIST_BEGIN()
	// Objects
	CCL_OBJLIST_ENTRY	(Dictionary)
	CCL_OBJLIST_ENTRY	(List)
	CCL_OBJLIST_ENTRY	(Queue)
	CCL_OBJLIST_ENTRY	(Stack)

	// Nodes
	CCL_OBJLIST_ENTRY	(Iterate)
	CCL_OBJLIST_ENTRY	(Keys)
	CCL_OBJLIST_ENTRY	(Load)
	CCL_OBJLIST_ENTRY	(Remove)
	CCL_OBJLIST_ENTRY	(Store)
	CCL_OBJLIST_ENTRY	(Stat)
	CCL_OBJLIST_ENTRY	(Write)
CCL_OBJLIST_END()
