////////////////////////////////////////////////////////////////////////
//
//									VideoCapture.CPP
//
//				Implementation of the video capture node.
//
////////////////////////////////////////////////////////////////////////

#include "imagel_.h"
#include <stdio.h>

// Globals

VideoCapture :: VideoCapture ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pImg	= NULL;
	bOpen = false;
	}	// VideoCapture

HRESULT VideoCapture :: onAttach ( bool bAttach )
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
//		if (pnDesc->load ( adtString(L"Location"), vL ) == S_OK)
//			adtValue::toString ( vL, strLoc );
		}	// if

	// Detach
	else
		{
		// Shutdown
		if (bOpen)
			{
			bOpen = false;
			vc.release();
			}	// if
		_RELEASE(pImg);
		}	// else

	return hr;
	}	// onAttach

HRESULT VideoCapture :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Grab video frame
	if (_RCP(Fire))
		{
		cvMatRef	*pMat	= NULL;

		// State check
		CCLTRYE ( bOpen == true && pImg != NULL, ERROR_INVALID_STATE );

		// Create a matrix reference object for the image
		CCLTRYE( (pMat = new cvMatRef()) != NULL, E_OUTOFMEMORY );
		CCLTRYE ( (pMat->mat = new cv::Mat()) != NULL, E_OUTOFMEMORY );

		// Capture a frame
		CCLTRYE ( vc.read(*(pMat->mat)) == true, E_UNEXPECTED );

		// Result
		CCLTRY(pImg->store ( adtString(L"cvMatRef"), adtIUnknown(pMat) ) );
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(pImg));
		else
			_EMT(Error,adtInt(hr));
		}	// if

	// Open
	else if (_RCP(Open))
		{
		// State check
		CCLTRYE ( bOpen == false, ERROR_INVALID_STATE );

		// Open
		if (hr == S_OK)
			{
			try
				{
				// TODO: Device specification
				vc.open(0);
				}	// try
			catch ( cv::Exception &ex )
				{
				lprintf ( LOG_WARN, L"%S\r\n", ex.err.c_str() );
				hr = E_UNEXPECTED;
				}	// catch
			}	// if

		// Result
		if (hr == S_OK)
			{
			bOpen = true;
			_EMT(Open,adtBool(bOpen));
			}	// if
		else
			_EMT(Error,adtInt(hr));
		}	// else if

	// Close
	else if (_RCP(Close))
		{
		// Shutdown
		if (bOpen)
			{
			bOpen = false;
			vc.release();
			}	// if

		// Result ?
//		_EMT(Open,adtBool(false));
		}	// else if

	// State
	else if (_RCP(Image))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pImg);
		_QISAFE(unkV,IID_IDictionary,&pImg);
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

