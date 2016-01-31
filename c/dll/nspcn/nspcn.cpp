////////////////////////////////////////////////////////////////////////
//
//									NSPCN.CPP
//
//				Main file for the nSpace node library
//
////////////////////////////////////////////////////////////////////////

#define	CCL_OBJ_PREFIX		L"nSpace"
#define	CCL_OBJ_MODULE		L"nSpc"
#include "../../lib/nspcl/nspcl_.h"

// Objects in this module
CCL_OBJLIST_BEGIN()

	// Objects
//	CCL_OBJLIST_ENTRY	(Graph)
	CCL_OBJLIST_ENTRY	(Location)
	CCL_OBJLIST_ENTRY	(Namespace)
//	CCL_OBJLIST_ENTRY	(Node)
	CCL_OBJLIST_ENTRY	(PersistTxt)
	CCL_OBJLIST_ENTRY	(TemporalImpl)

	// Nodes
	CCL_OBJLIST_ENTRY	(Connectors)
	CCL_OBJLIST_ENTRY	(KeyPath)
	CCL_OBJLIST_ENTRY	(Link)
	CCL_OBJLIST_ENTRY	(Reflect)
	CCL_OBJLIST_ENTRY	(Temporal)
	CCL_OBJLIST_ENTRY	(This)
	CCL_OBJLIST_ENTRY	(Value)
//	CCL_OBJLIST_ENTRY	(Values)

CCL_OBJLIST_END()
