////////////////////////////////////////////////////////////////////////
//
//									VALUE.CPP
//
//					Implementation of the value node
//
////////////////////////////////////////////////////////////////////////

#include "nspcl_.h"

// String references
static adtString	strRefValue		( L"Value" );
static adtString	strRefType		( L"Type" );
static adtString	strRefMin		( L"Minimum" );
static adtString	strRefMax		( L"Maximum" );
static adtString	strRefAllowed	( L"Allowed" );

Value :: Value ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pDsc = NULL;
	}	// Value

HRESULT Value :: onAttach ( bool bAttach )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when this behaviour is assigned to a node
	//
	//	PARAMETERS
	//		-	bAttach is true for attachment, false for detachment.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;

	// Attach
	if (bAttach)
		{
		adtValue		vL;

		// Default values
		if (	pnDesc->load ( strRefValue, vL )	== S_OK )
			receive ( prFire, L"Value", vL );
		if (	pnDesc->load ( strRefType, vL )	== S_OK )
			strType = vL;

		// The descriptor is emitted for this value so the outside world
		// can monitor value attributes (minimum,maximum,type,etc).
		pDsc = pnDesc;
		_ADDREF(pDsc);
		_EMT(Descriptor,adtIUnknown(pDsc));
		}	// if

	else
		{
		_RELEASE(pDsc);
		}	// else
	return hr;
	}	// onAttach

HRESULT Value :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	FROM	IReceptor
	//
	//	PURPOSE
	//		-	A location has received a value on the specified receptor.
	//
	//	PARAMETERS
	//		-	pr is the receptor
	//		-	pl is the location
	//		-	v is the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Value
	if (_RCP(Fire))
		{
		// Validate
		if (strType[0] != '\0')
			validate ( pDsc, v, vE );
		else
			adtValue::copy ( v, vE );

		// Updated value
		_EMT(Fire,vE);
		}	// if

	// Descriptor
	else if (_RCP(Descriptor))
		{
		adtIUnknown unkV(v);
		_RELEASE(pDsc);
		_QISAFE(unkV,IID_IDictionary,&pDsc);
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

HRESULT Value :: validate ( IDictionary *pAttr, const ADTVALUE &v, 
										ADTVALUE &vV )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Validate and possibly adjust the value based on the
	//			specifications in the provided attributes.
	//
	//	PARAMETERS
	//		-	pAttr are the node attributes
	//		-	v contains the source value
	//		-	vV will receive the validated value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr = S_OK;

	// Default to original value
	CCLTRY ( adtValue::copy ( v, vV ) );

	// Type check
	if (strType[0] != '\0')
		{
		// String
		if (!WCASECMP(strType,L"String"))
			{
			// Convert value to a string
			CCLTRY ( adtValue::toString ( v, strV ) );
			CCLTRY ( adtValue::copy ( strV, vV ) );
			}	// if

		// Integer
		else if (!WCASECMP(strType,L"Integer"))
			hr = adtValue::copy ( adtInt(v), vV );

		// Long
		else if (!WCASECMP(strType,L"Long"))
			hr = adtValue::copy ( adtLong(v), vV );

		// Float
		else if (!WCASECMP(strType,L"Float"))
			hr = adtValue::copy ( adtFloat(v), vV );

		// Double
		else if (!WCASECMP(strType,L"Double"))
			hr = adtValue::copy ( adtDouble(v), vV );

		// Date
		else if (!WCASECMP(strType,L"Date"))
			hr = adtValue::copy ( adtDate(v), vV );

		// Boolean
		else if (!WCASECMP(strType,L"Boolean"))
			hr = adtValue::copy ( adtBool(v), vV );


		// Limits
		if (hr == S_OK)
			{
			// Compare based on type
			switch ( vV.vtype )
				{
				case VTYPE_I4 :
					if (	pAttr->load ( strRefMin, vL ) == S_OK	&&
							!adtValue::empty(vL)											&&
							vV.vint < adtInt(vL).vint )
						vV.vint = adtInt(vL);
					if (	pAttr->load ( strRefMax, vL ) == S_OK		&&
							!adtValue::empty(vL)											&&
							vV.vint > adtInt(vL).vint )
						vV.vint = adtInt(vL);
					break;
				case VTYPE_I8 :
					if (	pAttr->load ( strRefMin, vL ) == S_OK	&&
							!adtValue::empty(vL)											&&
							vV.vlong < adtLong(vL).vlong )
						vV.vlong = adtLong(vL);
					if (	pAttr->load ( strRefMax, vL ) == S_OK	&&
							!adtValue::empty(vL)											&&
							vV.vlong > adtLong(vL).vlong )
						vV.vlong = adtLong(vL);
					break;
				case VTYPE_R4 :
					if (	pAttr->load ( strRefMin, vL ) == S_OK	&&
							!adtValue::empty(vL)											&&
							vV.vflt < adtFloat(vL) )
						vV.vflt = adtFloat(vL);
					if (	pAttr->load ( strRefMax, vL ) == S_OK	&&
							!adtValue::empty(vL)											&&
							vV.vflt > adtFloat(vL) )
						vV.vflt = adtFloat(vL);
					break;
				case VTYPE_R8 :
					if (	pAttr->load ( strRefMin, vL ) == S_OK	&&
							!adtValue::empty(vL)											&&
							vV.vdbl < adtDouble(vL) )
						vV.vdbl = adtDouble(vL);
					if (	pAttr->load ( strRefMax, vL ) == S_OK	&&
							!adtValue::empty(vL)											&&
							vV.vdbl > adtDouble(vL) )
						vV.vdbl = adtDouble(vL);
					break;
				case VTYPE_DATE :
					if (	pAttr->load ( strRefMin, vL ) == S_OK	&&
							!adtValue::empty(vL)											&&
							vV.vdate < adtDate(vL) )
						vV.vdate = adtDate(vL);
					if (	pAttr->load ( strRefMax, vL ) == S_OK	&&
							!adtValue::empty(vL)											&&
							vV.vdate > adtDate(vL) )
						vV.vdate = adtDate(vL);
					break;
				case VTYPE_STR :
//				case VTYPE_STR|VTYPE_BYREF :
				case VTYPE_STR|VTYPE_CONST :
					// Allowed strings ?
					if (pAttr->load ( strRefAllowed, vL ) == S_OK)
						{
						IContainer	*pCont	= NULL;
						IIt			*pIt		= NULL;
						bool			bMatch	= false;
						adtIUnknown	unkV(vL);
						adtValue		vFrst;

						// Iterate allowed values
						CCLTRY ( _QISAFE(unkV,IID_IContainer,&pCont) );
						CCLTRY ( pCont->iterate ( &pIt ) );
						while (hr == S_OK && !bMatch && pIt->read ( vL ) == S_OK)
							{
							// Store first value in case of no match
							if (adtValue::empty(vFrst))
								adtValue::copy ( vL, vFrst );

							// Match ?
							bMatch = (adtValue::compare ( vL, vV ) == 0);

							// Clean up
							pIt->next();
							}	// while

						// If no match use default or first value
						if (hr == S_OK && !bMatch)
							{
							if (pAttr->load ( strRefValue, vL ) == S_OK)
								hr = adtValue::copy ( vL, vV );
							else if (!adtValue::empty(vFrst))
								hr = adtValue::copy ( vFrst, vV );
							}	// if

						// Clean up
						_RELEASE(pIt);
						_RELEASE(pCont);
						}	// if
					break;
				}	// switch
			}	// if

		}	// if

	return S_OK;
	}	// validate

