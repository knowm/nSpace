////////////////////////////////////////////////////////////////////////
//
//									NETN.CPP
//
//					Main file for the IO library
//
////////////////////////////////////////////////////////////////////////

#define	CCL_OBJ_PREFIX		L"nSpace"
#define	CCL_OBJ_MODULE		L"Net"

// Library implementations
#include "../../lib/netl/netl_.h"

// Objects in this module
CCL_OBJLIST_BEGIN()
	// Objects

	// Nodes
	CCL_OBJLIST_ENTRY	(Address)
	CCL_OBJLIST_ENTRY	(Avail)
	CCL_OBJLIST_ENTRY	(Client)
	CCL_OBJLIST_ENTRY	(DatagramOp)
	CCL_OBJLIST_ENTRY	(Interfaces)
	CCL_OBJLIST_ENTRY	(MulticastOp)
	CCL_OBJLIST_ENTRY	(PersistSkt)
	CCL_OBJLIST_ENTRY	(Recv)
	CCL_OBJLIST_ENTRY	(Send)
	CCL_OBJLIST_ENTRY	(SocketOp)
	#ifdef	USE_WEBSKTPP
	CCL_OBJLIST_ENTRY	(WebSktSrvr)
	#endif
CCL_OBJLIST_END()

