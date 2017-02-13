////////////////////////////////////////////////////////////////////////
//
//									BOOL.CPP
//
//					Implementation of the boolean value class
//
////////////////////////////////////////////////////////////////////////

#include "adtl.h"

adtBool :: adtBool ( bool val )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	//	PARAMETERS
	//		-	val is the initial value
	//
	////////////////////////////////////////////////////////////////////////
	vtype	= VTYPE_BOOL;
	vbool	= (val) ? TRUE : FALSE;
	}	// adtBool

adtBool :: adtBool ( const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	//	PARAMETERS
	//		-	v is the value to initialize with
	//
	////////////////////////////////////////////////////////////////////////
	vtype	= VTYPE_BOOL;
	vbool	= FALSE;
	if (v.vtype == VTYPE_BOOL)
		vbool = v.vbool;
	}	// adtBool

//
// Operators
//

adtBool& adtBool::operator= ( const ADTVALUE &v )
	{
	vtype	= VTYPE_BOOL;
	if			(v.vtype == VTYPE_BOOL)						vbool = v.vbool;
	else if	(v.vtype == VTYPE_I4)						vbool	= (v.vint != 0);
	else if	(v.vtype == VTYPE_I8)						vbool	= (v.vlong != 0);
	else if	(v.vtype == VTYPE_R4)						vbool	= (v.vflt != 0);
	else if	(v.vtype == VTYPE_R8)						vbool	= (v.vdbl != 0);
	else if	(v.vtype == VTYPE_DATE)						vbool	= (v.vdate != 0);
	else if	(	v.vtype == (VTYPE_VALUE|VTYPE_BYREF) &&
					v.pval != NULL)							*this = *(v.pval);
	else															vbool	= false;
	return *this;
	}	// operator=

