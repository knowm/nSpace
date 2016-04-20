////////////////////////////////////////////////////////////////////////
//
//									TOGGLE.CPP
//
//					Implementation of the toggle node.
//
////////////////////////////////////////////////////////////////////////

#include "miscl_.h"
#include <stdio.h>

Toggle :: Toggle ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	bVal = false;
	}	// Toggle

HRESULT Toggle :: onAttach ( bool bAttach )
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
	pnDesc->load ( adtString(L"Default"), bVal );

	return S_OK;
	}	// onAttach

HRESULT Toggle :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

//if (!WCASECMP(L"OnNodeLink",this->strnName))
//	dbgprintf ( L"Hi\r\n" );

	// Fire
	if (_RCP(Fire))
		{
		// Current value must be cached in case it changes during emission
		bValE = bVal;

		// Emit state
		if (bValE== true)	_EMT(True,v);
		else					_EMT(False,v);
		_EMT(Fire,bValE);
		}	// if

	// Invert state
	else if (_RCP(Not))
		bVal = (bVal == true) ? false : true;

	// Context
	else if (_RCP(False))
		bVal = false;
	else if (_RCP(True))
		bVal = true;
	else if (_RCP(Value))
		bVal = adtBool(v);
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

