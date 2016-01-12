////////////////////////////////////////////////////////////////////////
//
//									INT.CPP
//
//					Implementation of the integer value class
//
////////////////////////////////////////////////////////////////////////

#include "adtl.h"

adtInt :: adtInt ( U32 val )
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
	vtype	= VTYPE_I4;
	vint	= val;
	}	// adtInt

adtInt :: adtInt ( const ADTVALUE &v )
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
	vtype	= VTYPE_I4;
	vint	= 0;
	*this = v;
	}	// adtInt

//
// Operators
//

adtInt& adtInt::operator= ( const ADTVALUE &v )
	{
	adtValue::clear(*this);
	vtype	= VTYPE_I4;
	if			(adtValue::type(v) == VTYPE_I4)		vint	= v.vint;
	else if	(adtValue::type(v) == VTYPE_I8)		vint	= (U32) v.vlong;
	else if	(adtValue::type(v) == VTYPE_R4)		vint	= (U32) v.vflt;
	else if	(adtValue::type(v) == VTYPE_R8)		vint	= (U32) v.vdbl;
	else if	(adtValue::type(v) == VTYPE_DATE)	vint	= (U32) v.vdate;
	else if	(adtValue::type(v) == VTYPE_BOOL)	vint	= (v.vbool == TRUE);
	else if	(adtValue::type(v) == VTYPE_STR)		adtValue::fromString ( v.pstr, VTYPE_I4, *this );
	else if	(	v.vtype == (VTYPE_VALUE|VTYPE_BYREF) &&
					v.pval != NULL)						*this = *(v.pval);
	else														vint	= 0;
	return *this;
	}	// operator=

adtInt& adtInt::operator= ( const U32 v )
	{
	adtValue::clear(*this);
	vtype	= VTYPE_I4;
	vint	= v;
	return *this;
	}	// operator=
