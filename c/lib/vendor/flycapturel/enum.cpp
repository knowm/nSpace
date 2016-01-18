////////////////////////////////////////////////////////////////////////
//
//									ENUM.CPP
//
//		Implementation of the Point Grey camera node
//
////////////////////////////////////////////////////////////////////////

#include "flycapturel_.h"
#include <stdio.h>

using namespace FlyCapture2;

// Globals

Enum :: Enum ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	iIdx	= 0;
	}	// Enum

HRESULT Enum :: onAttach ( bool bAttach )
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
		}	// if

	// Detach
	else
		{
		}	// else

	return hr;
	}	// onAttach

HRESULT Enum :: receive ( IReceptor *pr, const WCHAR *pl, 
										const ADTVALUE &v )
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

	// First/next camera
	if (_RCP(First) || _RCP(Next))
		{
		IDictionary	*pCam	= NULL;
		bool			bConn	= false;
		BusManager	bus;
		U32			iNum;
		PGRGuid		id;

		// First
		iIdx = (_RCP(First)) ? 0 : iIdx+1;

		// Retrieve the number of cameras in system
		CCLTRY ( pgrError ( bus.GetNumOfCameras ( &iNum ) ) );

		// State check
		CCLTRYE ( iIdx >= 0 && iIdx < iNum, ERROR_NOT_FOUND );

		// Retrieve the Id for the camera
		CCLTRY ( pgrError ( bus.GetCameraFromIndex ( iIdx, &id ) ) );

		// Result
		if (hr == S_OK)
			_EMT(Next,adtInt(iIdx));
		else
			_EMT(End,adtInt(iIdx));

		// Clean up
		_RELEASE(pCam);
		}	// if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive
