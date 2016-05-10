////////////////////////////////////////////////////////////////////////
//
//									RESOURCE.CPP
//
//					Implementation of the generic resource node
//
////////////////////////////////////////////////////////////////////////

#include "iol_.h"
#include <stdio.h>

Resource :: Resource ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pOpt	= NULL;	
	pRes	= NULL;
	}	// Resource

void Resource :: destruct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		CCLObject
	//
	//	PURPOSE
	//		-	Called when the object is being destroyed
	//
	////////////////////////////////////////////////////////////////////////
	_RELEASE(pRes);
	_RELEASE(pOpt);
	}	// destruct

HRESULT Resource :: onAttach ( bool bAttach )
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
	adtIUnknown		unkV;

	// State check
	if (!bAttach) return S_OK;

	// Default options
	if (pnDesc->load ( adtString(L"Options"), unkV ) == S_OK)
		{
		CCLTRY(_QISAFE(unkV,IID_IDictionary,&pOpt));
		}	// if
	else
		{
		CCLTRY(COCREATE(L"Adt.Dictionary",IID_IDictionary,&pOpt));
		}	// else

	return S_OK;
	}	// onAttach

HRESULT Resource :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

	// Open
	if (_RCP(Open))
		{
		// State check
		CCLTRYE ( pRes != NULL, ERROR_INVALID_STATE );
		CCLTRYE ( pOpt != NULL, ERROR_INVALID_STATE );

		// Access
		CCLTRY ( pRes->open ( pOpt ) );

		// Result
		if (hr == S_OK)
			_EMT(Open,adtIUnknown(pRes) );
		else
			_EMT(Error,adtInt(hr) );
		}	// if

	// Close
	else if (_RCP(Close))
		{
		// State check
		CCLTRYE ( pRes != NULL, ERROR_INVALID_STATE );

		// Access
		lprintf ( LOG_INFO, L"Close:pRes %p\r\n", pRes );
		CCLTRY ( pRes->close() );
		}	// else if

	// State
	else if (_RCP(Options))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pOpt);
		hr = _QI(unkV,IID_IDictionary,&pOpt);
		}	// else if
	else if (_RCP(Resource))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pRes);
		hr = _QISAFE(unkV,IID_IResource,&pRes);
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

