////////////////////////////////////////////////////////////////////////
//
//									LONG.CPP
//
//					Implementation of the long value class
//
////////////////////////////////////////////////////////////////////////

#include "adtl.h"

adtLong :: adtLong ( U64 val )
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
	vtype	= VTYPE_I8;
	vlong	= val;
	}	// adtLong

adtLong :: adtLong ( const ADTVALUE &v )
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
	vtype	= VTYPE_I8;
	vlong	= 0;
	*this = v;
	}	// adtLong

//
// Operators
//

adtLong& adtLong::operator= ( const ADTVALUE &v )
	{
	adtValue::clear(*this);
	vtype	= VTYPE_I8;
	if			(adtValue::type(v) == VTYPE_I4)		vlong	= v.vint;
	else if	(adtValue::type(v) == VTYPE_I8)		vlong	= v.vlong;
	else if	(adtValue::type(v) == VTYPE_R4)		vlong	= (U64) v.vflt;
	else if	(adtValue::type(v) == VTYPE_R8)		vlong	= (U64) v.vdbl;
	else if	(adtValue::type(v) == VTYPE_DATE)		vlong	= (U64) v.vdate;
	else if	(adtValue::type(v) == VTYPE_BOOL)		vlong	= (v.vbool == TRUE);
	else if	(adtValue::type(v) == VTYPE_STR)		adtValue::fromString ( v.pstr, VTYPE_I8, *this );
	else if	(	v.vtype == (VTYPE_VALUE|VTYPE_BYREF) &&
					v.pval != NULL)						*this = *(v.pval);
	return *this;
	}	// operator=

adtLong& adtLong::operator= ( const U64 v )
	{
	adtValue::clear(*this);
	vtype	= VTYPE_I8;
	vlong	= v;
	return *this;
	}	// operator=
