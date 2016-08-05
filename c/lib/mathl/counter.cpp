////////////////////////////////////////////////////////////////////////
//
//									COUNTER.CPP
//
//					Implementation of the counter node
//
////////////////////////////////////////////////////////////////////////

#include "mathl_.h"
#include <stdio.h>

Counter :: Counter ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	vReset	= 0;
	}	// Counter

HRESULT Counter :: onAttach ( bool bAttach )
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

	// State check
	if (!bAttach) return S_OK;

	// Default states
	pnDesc->load ( adtString(L"Reset"),	vReset );

	return S_OK;
	}	// onAttach

HRESULT Counter :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Increment
	if (_RCP(Increment) || _RCP(Decrement))
		{
		// Update value
		vCnt	= _RCP(Increment) ? vCnt+1 : vCnt-1;

		// Emit
		_EMT(Fire,vCnt);
		}	// if

	// Reset
	else if (_RCP(Reset))
		{
		// Update value
		vCnt = (U32) vReset;

		// Emit
		_EMT(Fire,vCnt);
		}	// else if

	// Reset value
	else if (_RCP(Fire))
		hr = adtValue::copy ( adtInt(v), vReset );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

