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
	bRun		= false;
	evEmit.init();
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
		adtValue vL;

		// Default states
		pnDesc->load ( adtString(L"Value"), vVal );
		}	// if

	// Detach
	else if (!bAttach)
		{
		// Shutdown thread
		if (pThrd != NULL)
			{
			bRun = false;
			pThrd->threadStop(5000);
			_RELEASE(pThrd);
			}	// if
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
		// Value slot available ?
		CCLTRYE ( adtValue::empty(vEmit) == true, ERROR_INVALID_STATE );

		// Create thread if necessary
		if (hr == S_OK && pThrd == NULL)
			{
			CCLTRY(COCREATE(L"Sys.Thread", IID_IThread, &pThrd ));
			CCLOK (bRun = true;)
			CCLTRY(pThrd->threadStart ( this, 5000 ));
			}	// if

		// Copy value to emit
		CCLTRY ( adtValue::copy ( adtValue::empty(vVal) ? v : vVal, vEmit ) );

		// Emission value available
		CCLOK ( evEmit.signal(); )
		}	// if

	// State
	else if (_RCP(Value))
		adtValue::copy ( v, vVal );
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
	HRESULT		hr = S_OK;
	adtValue		vEmitNow;

	// Wait for value to emit
	CCLTRYE ( evEmit.wait(-1), ERROR_TIMEOUT );

	// Still running ?
	CCLTRYE ( bRun == true, S_FALSE );

	// Emit value
	if (hr == S_OK)
		{
		// Make local copy so value can be readied immediately
		CCLTRY ( adtValue::copy ( vEmit, vEmitNow ) );

		// Clear value to signal slot available
		CCLOK ( adtValue::clear ( vEmit ); )

		// Emit
		_EMT(Fire,vEmitNow);
		}	// if

	return hr;
	}	// tick

