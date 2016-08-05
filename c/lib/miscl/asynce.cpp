////////////////////////////////////////////////////////////////////////
//
//									ASYNCE.CPP
//
//				Implementation of the asynchronous emission node
//
////////////////////////////////////////////////////////////////////////

#include "miscl_.h"
#include <stdio.h>

AsyncEmit :: AsyncEmit ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the CD Player node
	//
	////////////////////////////////////////////////////////////////////////
	pThrd		= NULL;
	bEmit		= false;
	}	// AsyncEmit

void AsyncEmit :: destruct ( void )
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

HRESULT AsyncEmit :: onAttach ( bool bAttach )
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
		// Default states
		pnDesc->load ( adtString(L"Value"), vVal );
		}	// if

	// Detach
	else if (!bAttach)
		{
		// Shutdown thread
		_RELEASE(pThrd);
		}	// else if

	return hr;
	}	// onAttach

HRESULT AsyncEmit :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Emit
	if (_RCP(Fire))
		{
		// Already emitting?
		CCLTRYE ( bEmit == false, E_UNEXPECTED );

		// Copy value to emit
		CCLTRY ( adtValue::copy ( adtValue::empty(vVal) ? v : vVal, vEmit ) );

		// Start emission
		if (hr == S_OK)
			{
			_RELEASE(pThrd);
			CCLTRY(COCREATE(L"Sys.Thread", IID_IThread, &pThrd ));
			CCLOK (bEmit = true;)
			CCLTRY(pThrd->threadStart ( this, 5000 ));
			}	// if
		}	// if

	// State
	else if (_RCP(Value))
		adtValue::copy ( v, vEmit );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

HRESULT AsyncEmit :: tick ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ITickable
	//
	//	PURPOSE
	//		-	Perform one 'tick's worth of work.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	// Debug
//	Sleep(2000);

	// Emit value
	_EMT(Fire,vEmit);
	bEmit			= false;

	// Asynchronous emitter is a single shot event
	return S_FALSE;
	}	// tick

