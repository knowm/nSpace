////////////////////////////////////////////////////////////////////////
//
//									DOUBLE.CPP
//
//		Implementation of the double precision floating point value class
//
////////////////////////////////////////////////////////////////////////

#include "adtl.h"

adtDouble :: adtDouble ( double dval )
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
	vtype	= VTYPE_R8;
	vdbl	= dval;
	}	// adtDouble

adtDouble :: adtDouble ( float dval )
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
	vtype	= VTYPE_R8;
	vdbl	= dval;
	}	// adtDouble

adtDouble :: adtDouble ( const ADTVALUE &v )
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
	vtype	= VTYPE_R8;
	vdbl	= 0.0;
	*this	= v;
	}	// adtDouble

//
// Operators
//

adtDouble& adtDouble::operator= ( const ADTVALUE &v )
	{
	adtValue::clear(*this);
	vtype	= VTYPE_R8;
	if			(adtValue::type(v) == VTYPE_R8)	vdbl = v.vdbl;
	else if	(adtValue::type(v) == VTYPE_R4)	vdbl = v.vflt;
	else if	(adtValue::type(v) == VTYPE_I4)	vdbl = (double)(v.vint);
	else if	(adtValue::type(v) == VTYPE_I8)	vdbl = (double)(v.vlong);
	else if	(adtValue::type(v) == VTYPE_STR)	adtValue::fromString ( v.pstr, VTYPE_R8, *this );
	else if	(	v.vtype == (VTYPE_VALUE|VTYPE_BYREF) &&
					v.pval != NULL)					*this = *(v.pval);
	else									vdbl = 0.0;
	return *this;
	}	// operator=

