////////////////////////////////////////////////////////////////////////
//
//									MATHN.CPP
//
//					Main file for the math library
//
////////////////////////////////////////////////////////////////////////

#define	CCL_OBJ_PREFIX		L"nSpace"
#define	CCL_OBJ_MODULE		L"Math"

// Library implementations
#include "../../lib/mathl/mathl_.h"

// Objects in this module
CCL_OBJLIST_BEGIN()

	// Objects

	// Nodes
	CCL_OBJLIST_ENTRY	(Binary)
	CCL_OBJLIST_ENTRY	(Counter)
	CCL_OBJLIST_ENTRY	(Formula)
//	CCL_OBJLIST_ENTRY	(Function)
	CCL_OBJLIST_ENTRY	(Matrix3D)
	CCL_OBJLIST_ENTRY	(Transform3D)
//	CCL_OBJLIST_ENTRY	(Vector3)
	CCL_OBJLIST_ENTRY	(Unary)

CCL_OBJLIST_END()
