////////////////////////////////////////////////////////////////////////
//
//									ION.CPP
//
//					Main file for the IO library
//
////////////////////////////////////////////////////////////////////////

#define	INITGUID
#define	CCL_OBJ_PREFIX		L"nSpace"
#define	CCL_OBJ_MODULE		L"IO"

// Library implementations
#include "../../lib/iol/iol_.h"

// Objects in this module
CCL_OBJLIST_BEGIN()
	// Objects
	CCL_OBJLIST_ENTRY	(ByteCache)
	CCL_OBJLIST_ENTRY	(Lock)
	CCL_OBJLIST_ENTRY	(MemoryBlock)
	CCL_OBJLIST_ENTRY	(StmSrcFile)
	CCL_OBJLIST_ENTRY	(StmFile)
	CCL_OBJLIST_ENTRY	(StmMemory)
	CCL_OBJLIST_ENTRY	(StmPrsBin)
	CCL_OBJLIST_ENTRY	(StmPrsXML)

	// Nodes
	#ifdef	_WIN32
	CCL_OBJLIST_ENTRY	(EnumDevClass)
	#endif
	CCL_OBJLIST_ENTRY	(Resource)
	CCL_OBJLIST_ENTRY	(Persist)
	CCL_OBJLIST_ENTRY	(StreamOp)
	CCL_OBJLIST_ENTRY	(StreamCopy)
	CCL_OBJLIST_ENTRY	(StreamSource)
CCL_OBJLIST_END()
