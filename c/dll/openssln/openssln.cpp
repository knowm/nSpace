////////////////////////////////////////////////////////////////////////
//
//									OPENSSLN.CPP
//
//					Main file for the OpenSSL node library
//
////////////////////////////////////////////////////////////////////////

#define	INITGUID
#define	CCL_OBJ_PREFIX		L"nSpace"
#define	CCL_OBJ_MODULE		L"OpenSSL"

// Library implementations
#include "../../lib/openssll/openssll_.h"

// Objects in this module
CCL_OBJLIST_BEGIN()

	// Objects

	// Nodes
	CCL_OBJLIST_ENTRY	(EVPSign)
	CCL_OBJLIST_ENTRY	(EVPVerify)
	CCL_OBJLIST_ENTRY	(PEMImpl)
	CCL_OBJLIST_ENTRY	(RSAImpl)
	CCL_OBJLIST_ENTRY	(SSLConnect)

CCL_OBJLIST_END()
