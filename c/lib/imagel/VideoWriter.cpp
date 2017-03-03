////////////////////////////////////////////////////////////////////////
//
//									VideoWriter.CPP
//
//				Implementation of the video writer node.
//
////////////////////////////////////////////////////////////////////////

#include "imagel_.h"
#include <stdio.h>

// Globals

VideoWriter :: VideoWriter ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	iFps	= 10;
	iW		= 320;
	iH		= 240;
	}	// VideoWriter

HRESULT VideoWriter :: onAttach ( bool bAttach )
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
		adtValue		vL;

		// Defaults (optional)
		if (pnDesc->load ( adtString(L"FPS"), vL ) == S_OK)
			iFps = vL;
		if (pnDesc->load ( adtString(L"Width"), vL ) == S_OK)
			iW = vL;
		if (pnDesc->load ( adtString(L"Height"), vL ) == S_OK)
			iH = vL;
		}	// if

	// Detach
	else
		{
		// Shutdown
		}	// else

	return hr;
	}	// onAttach

HRESULT VideoWriter :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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
/*
	// Grab video frame
	if (_RCP(Fire))
		{
		// State check
		CCLTRYE ( bOpen == true, ERROR_INVALID_STATE );

		// Result
//		if (hr == S_OK)
//			_EMT(Fire,adtIUnknown(pImgUse));
//		else
//			_EMT(Error,adtInt(hr));

		}	// if

	// State
	else
*/		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

