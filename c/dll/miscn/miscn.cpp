////////////////////////////////////////////////////////////////////////
//
//									MISCN.CPP
//
//				Main file for the miscellaneous node library
//
////////////////////////////////////////////////////////////////////////

#define	CCL_OBJ_PREFIX		L"nSpace"
#define	CCL_OBJ_MODULE		L"Misc"
#include "../../lib/miscl/miscl_.h"

// Objects in this module
CCL_OBJLIST_BEGIN()

	// Nodes
	CCL_OBJLIST_ENTRY	(AsyncEmit)
	CCL_OBJLIST_ENTRY	(AsyncQ)
	CCL_OBJLIST_ENTRY	(Clone)
	CCL_OBJLIST_ENTRY	(Compare)
	CCL_OBJLIST_ENTRY	(Create)
	CCL_OBJLIST_ENTRY	(Decode)
	CCL_OBJLIST_ENTRY	(Debug)
	CCL_OBJLIST_ENTRY	(Demux)
	CCL_OBJLIST_ENTRY	(DictFormat)
	CCL_OBJLIST_ENTRY	(DictParse)
	CCL_OBJLIST_ENTRY	(Dist)
	CCL_OBJLIST_ENTRY	(Path)
	CCL_OBJLIST_ENTRY	(StringOp)
	CCL_OBJLIST_ENTRY	(StringFormat)
	CCL_OBJLIST_ENTRY	(StringStream)
	CCL_OBJLIST_ENTRY	(StringParse)
	CCL_OBJLIST_ENTRY	(TimeOp)
	CCL_OBJLIST_ENTRY	(Timer)
	CCL_OBJLIST_ENTRY	(Toggle)
	CCL_OBJLIST_ENTRY	(TokenIt)
	CCL_OBJLIST_ENTRY	(Type)
	CCL_OBJLIST_ENTRY	(UUIDOp)

CCL_OBJLIST_END()
