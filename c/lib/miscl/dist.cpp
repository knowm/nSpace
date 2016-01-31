////////////////////////////////////////////////////////////////////////
//
//									DIST.CPP
//
//					Implementation of the distribution node
//
////////////////////////////////////////////////////////////////////////

#define	INITGUID
#include "miscl_.h"

Dist :: Dist ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	}	// Dist

HRESULT Dist :: onAttach ( bool bAttach )
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
	HRESULT			hr		= S_OK;

	// Attach
	if (bAttach)
		{
		// A default value to emit can be specified
		pnDesc->load(adtString(L"Value"),vE);
		}	// if

	return hr;
	}	// onAttach

HRESULT Dist :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

	// Debug
//	if (!WCASECMP(strnName,L"ValueLocation"))
//		dbgprintf ( L"%s::Dist::receive\r\n", (LPCWSTR)strnName );

	// Fire
	if (_RCP(Fire))
		hr = _EMT(Fire, (!adtValue::empty(vE)) ? vE : v );

	// Value
	else if (_RCP(Value))
		hr = adtValue::copy ( v, vE );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

