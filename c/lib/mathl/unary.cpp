////////////////////////////////////////////////////////////////////////
//
//									UNARY.CPP
//
//					Implementation of the unary operation node
//
////////////////////////////////////////////////////////////////////////

#include "mathl_.h"
#include <stdio.h>

Unary :: Unary ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////

	// Default is no-op
	iOp	 = MATHOP_NOP;
	}	// Unary

HRESULT Unary :: onAttach ( bool bAttach )
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

	// Default states
	pnDesc->load ( adtStringSt(L"Value"), vV );
	if (	pnDesc->load ( adtStringSt(L"Op"), v ) == S_OK	&& 
			adtValue::type(v) == VTYPE_STR						&&
			v.pstr != NULL )
		mathOp ( v.pstr, &iOp );

	return S_OK;
	}	// onAttach

HRESULT Unary :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

	// Execute
	if (_RCP(Fire))
 		{
		const ADTVALUE *pvUse	= (!adtValue::empty(vV)) ? &vV : &v;

		// Perform operation
		CCLTRY ( mathUnary ( iOp, *pvUse, vRes ) );

		// Result
		if (hr == S_OK)
			_EMT(Fire,vRes);
		else
			{
			dbgprintf ( L"%s:Fire:Error:hr 0x%x:%d\r\n", (LPCWSTR)strnName, hr, iOp );
			_EMT(Error,adtInt(hr) );
			}	// else
		}	// if

	// State
	else if (_RCP(Value))
		hr = adtValue::copy ( v, vV, true );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive


