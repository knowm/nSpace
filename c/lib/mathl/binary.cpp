////////////////////////////////////////////////////////////////////////
//
//									BINARY.CPP
//
//					Implementation of the binary operation node
//
////////////////////////////////////////////////////////////////////////

#include "mathl_.h"
#include <stdio.h>

Binary :: Binary ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////

	// Default is add, most common.
	iOp	 = MATHOP_ADD;
	}	// Binary

HRESULT Binary :: onAttach ( bool bAttach )
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
	adtValue	v;

	// State check
	if (!bAttach) return S_OK;

	// Debug
//	if (!WCASECMP(this->strnName,L"MissAdd"))
//		dbgprintf ( L"Hi\r\n" );

	// Default states
	pnDesc->load ( adtStringSt(L"Left"), vL );
	pnDesc->load ( adtStringSt(L"Right"), vR );
	if (	pnDesc->load ( adtStringSt(L"Op"), v ) == S_OK	&& 
			adtValue::type(v) == VTYPE_STR						&&
			v.pstr != NULL )
		mathOp ( v.pstr, &iOp );

	return S_OK;
	}	// onAttach

HRESULT Binary :: onReceive ( IReceptor *pr, const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	The node has received a value on the specified receptor.
	//
	//	PARAMETERS
	//		-	pr is the receptor
	//		-	v is the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Debug
//	if (!WCASECMP(this->strnName,L"LightAvgX"))
//		dbgprintf ( L"Hi\r\n" );
	
	// Execute
	if (_RCP(Fire))
 		{
		// Debug
//		if (!WCASECMP(this->strnName,L"WidthDiv"))
//		if (iOp == MATHOP_SUB)
//		if (iOp == MATHOP_MOD)
//			dbgprintf ( L"Hi\r\n" );

		// State check
		CCLTRYE ( adtValue::empty(vL) == false, ERROR_INVALID_STATE );
		CCLTRYE ( adtValue::empty(vR) == false, ERROR_INVALID_STATE );
		CCLTRYE ( iOp >= MATHOP_ADD, ERROR_INVALID_STATE );

		// Perform operation, copy provided value in case operation 
		// needs a destination.
		CCLTRY ( adtValue::copy ( v, vRes ) );
		CCLTRY ( mathBinary ( iOp, vL, vR, vRes ) );

		// Result
		if (hr == S_OK)
			_EMT(Fire,vRes);
		else
			{
			lprintf ( LOG_ERR, L"%s:Fire:Error:hr 0x%x:%d\r\n", (LPCWSTR)strnName, hr, iOp );
			_EMT(Error,adtInt(hr) );
			}	// else
		}	// if

	// State
	else if (_RCP(Left))
		hr = adtValue::copy ( v, vL, true );
	else if (_RCP(Right))
		hr = adtValue::copy ( v, vR, true );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive


