////////////////////////////////////////////////////////////////////////
//
//									LOAD.CPP
//
//					Implementation of the load node
//
////////////////////////////////////////////////////////////////////////

#include "adtl_.h"

Load :: Load ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pDct	= NULL;
	}	// Load

void Load :: destruct ( void )
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
	onAttach(false);
	}	// destruct

HRESULT Load :: onAttach ( bool bAttach )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the behaviour is attached to a node.
	//
	//	PARAMETERS
	//		-	bAttach is true for attachment, false for detachment.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////

	// Attach
	if (bAttach)
		{
		pnDesc->load ( adtString(L"Key"),	vKey );
		}	// if

	// Detach
	else
		{
		_RELEASE(pDct);
		}	// else

	return S_OK;
	}	// onAttach

HRESULT Load :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Load value by key in context
	if (_RCP(Fire))
		{
		// State check
		CCLTRYE ( pDct != NULL,						ERROR_INVALID_STATE );
		CCLTRYE ( vKey.vtype != VTYPE_EMPTY,	ERROR_INVALID_STATE );

		// Debug
//		if (!WCASECMP(strnName,L"LoadImage"))
//			dbgprintf ( L"Hi\r\n" );

//		if (hr == S_OK)
//			{
//			adtString	strV;
//			adtValue::toString ( vKey, strV );
//			dbgprintf ( L"Load::receive:K:%s:%s\r\n", (LPCWSTR)strnName, (LPCWSTR) strV );
//			}	// if

		// Perform operation
		CCLTRY ( pDct->load ( vKey, vL ) );

		// Debug
//		if (hr == S_OK)
//			{
//			adtString	strV;
//			adtValue::toString ( vL, strV );
//			dbgprintf ( L"Load::receive:L:%s:%s\r\n", (LPCWSTR)strnName, (LPCWSTR) strV );
//			}	// if

		// Result
		if (hr == S_OK)
			_EMT(Fire,vL);
		else
			_EMT(NotFound,vKey);
		}	// if

	// Parameters
	else if (_RCP(Dictionary))
		{
		_RELEASE(pDct);
		hr = _QISAFE((unkV=v),IID_IDictionary,&pDct);
		}	// else if
	else if (_RCP(Key))
		hr = adtValue::copy ( v, vKey );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive
