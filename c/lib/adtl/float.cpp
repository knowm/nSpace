////////////////////////////////////////////////////////////////////////
//
//									FLOAT.CPP
//
//					Implementation of the floating point value class
//
////////////////////////////////////////////////////////////////////////

#include "adtl.h"

adtFloat :: adtFloat ( float fval )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	//	PARAMETERS
	//		-	fval is the initial value
	//
	////////////////////////////////////////////////////////////////////////
	vtype	= VTYPE_R4;
	vflt	= fval;
	}	// adtFloat

adtFloat :: adtFloat ( double dval )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	//	PARAMETERS
	//		-	dval is the initial value
	//
	////////////////////////////////////////////////////////////////////////
	vtype	= VTYPE_R4;
	vflt	= (float)dval;
	}	// adtFloat

adtFloat :: adtFloat ( const ADTVALUE &v )
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
	vtype	= VTYPE_R4;
	vflt	= 0.0f;
	*this	= v;
	}	// adtFloat

//
// Operators
//

adtFloat& adtFloat::operator= ( const ADTVALUE &v )
	{
	adtValue::clear(*this);
	vtype	= VTYPE_R4;
	if			(adtValue::type(v) == VTYPE_R4)	vflt = v.vflt;
	else if	(adtValue::type(v) == VTYPE_R8)	vflt = (float)(v.vdbl);
	else if	(adtValue::type(v) == VTYPE_I4)	vflt = (float)(v.vint);
	else if	(adtValue::type(v) == VTYPE_I8)	vflt = (float)(v.vlong);
	else if	(adtValue::type(v) == VTYPE_STR)	adtValue::fromString ( v.pstr, VTYPE_R4, *this );
	else if	(	v.vtype == (VTYPE_VALUE|VTYPE_BYREF) &&
					v.pval != NULL)					*this = *(v.pval);
	else									vflt = 0.0f;
	return *this;
	}	// operator=

