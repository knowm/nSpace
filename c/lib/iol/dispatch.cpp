////////////////////////////////////////////////////////////////////////
//
//									DISPATCH.CPP
//
//					Implementation of the IDispatch node
//
////////////////////////////////////////////////////////////////////////

#include "iol_.h"
#include <stdio.h>

Dispatch :: Dispatch ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pDct	= NULL;
	}	// Dispatch

void Dispatch :: destruct ( void )
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
	_RELEASE(pDct);
	}	// destruct

HRESULT Dispatch :: onAttach ( bool bAttach )
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

	// Attaching to graph
	if (bAttach)
		{
		}	// if

	// Detaching from graph
	else
		{
		// Clean up
		_RELEASE(pDct);
		}	// else

	return S_OK;
	}	// onAttach

HRESULT Dispatch :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Open/access object
	if (_RCP(Open))
		{
		IDispatch	*pDsp		= NULL;
		CLSID			clsid		= GUID_NULL;
		adtString	strId;
		adtValue		vL;

		// State check
		CCLTRYE ( pDct != NULL, ERROR_INVALID_STATE );

		// An object should not already be active
		CCLTRYE ( pDct->load ( adtString(L"Object"), vL ) != S_OK,
						ERROR_ALREADY_INITIALIZED );

		// An "Id" is required
		CCLTRY ( pDct->load ( adtString(L"Id"), vL ) );
		CCLTRYE( (strId = vL).length() > 0, ERROR_INVALID_STATE );

		// If a non-GUID is specified, convert it
		if (hr == S_OK && strId[0] != '{')
			hr = CLSIDFromProgID ( strId, &clsid );
		else
			hr = CLSIDFromString ( strId, &clsid );

		// Attempt to create the object on an IDispatch interface
		CCLTRY ( CoCreateInstance ( clsid, NULL, CLSCTX_ALL, IID_IDispatch, (void **) &pDsp ) );

		// Store results
		CCLTRY ( pDct->store ( adtString(L"Object"), adtIUnknown(pDsp) ) );

		// Result
		if (hr == S_OK)
			_EMT(Open,adtIUnknown(pDct));
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pDsp);
		}	// if

	// Close/release object
	else if (_RCP(Close))
		{
		// State check
		CCLTRYE ( pDct != NULL, ERROR_INVALID_STATE );

		// Closing
		_EMT(Close,adtIUnknown(pDct));

		// Remove active object
		CCLOK ( pDct->remove ( adtString(L"Object") ); )
		}	// if

	// State
	else if (_RCP(Dictionary))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pDct);
		hr = _QI(unkV,IID_IDictionary,&pDct);
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

