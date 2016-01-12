////////////////////////////////////////////////////////////////////////
//
//									IUNKNOWN.CPP
//
//					Implementation of the IUnknown class
//
////////////////////////////////////////////////////////////////////////

#include "adtl.h"

adtIUnknown :: adtIUnknown ( IUnknown *pUnk )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Default constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	vtype	= VTYPE_UNK;
	punk	= pUnk;
	if (punk) punk->AddRef();
	}	// adtIUnknown

adtIUnknown :: adtIUnknown ( const ADTVALUE &v )
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
	vtype	= VTYPE_UNK;
	punk	= NULL;
	*this	= v;
	}	// adtIUnknown

adtIUnknown :: ~adtIUnknown ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Destructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	adtValue::clear(*this);
	}	// ~adtIUnknown

//
// Operators
//

adtIUnknown& adtIUnknown::operator= ( const ADTVALUE &v )
	{
	adtValue::clear(*this);
	if (v.vtype == (VTYPE_VALUE|VTYPE_BYREF) && v.pval != NULL)
		*this = *(v.pval);
	else
		*this = (adtValue::type(v) == VTYPE_UNK) ? v.punk : (IUnknown *)NULL;
	return *this;
	}	// operator=

adtIUnknown& adtIUnknown::operator= ( IUnknown *pUnk )
	{
	adtValue::clear(*this);
	vtype	= VTYPE_UNK;
	punk	= pUnk;
	if (punk) punk->AddRef();
	return *this;
	}	// operator=

////////////////////
// 'adtIUnknownRef'
////////////////////

adtIUnknownRef :: adtIUnknownRef ( IUnknown *pUnk )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Default constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	vtype	= VTYPE_UNK|VTYPE_BYREF;
	punk	= pUnk;
	}	// adtIUnknownRef

adtIUnknownRef :: ~adtIUnknownRef ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Destructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	}	// ~adtIUnknownRef

//
// Operators
//

adtIUnknownRef& adtIUnknownRef::operator= ( IUnknown *pUnk )
	{
	adtValue::clear(*this);
	vtype		= VTYPE_UNK|VTYPE_BYREF;
	punk		= pUnk;
	return *this;
	}	// operator=

