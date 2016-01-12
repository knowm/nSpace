////////////////////////////////////////////////////////////////////////
//
//									Instance.CPP
//
//				Implementation of the WBEM Instance node
//
////////////////////////////////////////////////////////////////////////

#include "wmil_.h"
#include <stdio.h>

// Globals

Instance :: Instance ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pObj	= NULL;
	}	// Instance

HRESULT Instance :: onAttach ( bool bAttach )
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
	HRESULT	hr = S_OK;

	// Attach
	if (bAttach)
		{
		adtValue	vL;

		// Defaults
		if (pnDesc->load ( adtString(L"Key"), vL ) == S_OK)
			strKey = vL;
		}	// if

	// Detach
	else
		{
		_RELEASE(pObj);
		}	// else

	return hr;
	}	// onAttach

HRESULT Instance :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

	// Load
	if (_RCP(Load))
		{
		adtVariant	var;
		adtValue		vL;

		// State check
		CCLTRYE ( pObj != NULL, ERROR_INVALID_STATE );
		CCLTRYE ( strKey.length() > 0, ERROR_INVALID_STATE );

		// Retrieve value
		CCLTRY ( pObj->Get ( strKey, 0, &var, NULL, NULL ) );

		// Treat empty/null values as 'not found'
		CCLTRYE ( adtValue::empty((vL = var)) == false, ERROR_NOT_FOUND );

		// Result
		if (hr == S_OK)
			_EMT(Load,vL );
		else
			_EMT(NotFound,strKey );
		}	// if

	// State
	else if (_RCP(Object))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pObj);
		_QISAFE(unkV,IID_IWbemClassObject,&pObj);
		}	// else if
	else if (_RCP(Key))
		strKey = v;
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive
