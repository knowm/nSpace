////////////////////////////////////////////////////////////////////////
//
//									CONTOURS.CPP
//
//				Implementation of the image contours node.
//
////////////////////////////////////////////////////////////////////////

#include "imagel_.h"
#include <stdio.h>

// Globals

Contours :: Contours ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pImg	= NULL;
	iIdx	= 0;
	}	// Contours

HRESULT Contours :: onAttach ( bool bAttach )
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
//		if (pnDesc->load ( adtString(L"Size"), vL ) == S_OK)
//			iSz = vL;
		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pImg);
//		_RELEASE(pKer);
		}	// else

	return hr;
	}	// onAttach

HRESULT Contours :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

	// First/next
	if (_RCP(First) || _RCP(Next))
		{
		IDictionary	*pImgUse = NULL;
		cvMatRef		*pMat		= NULL;

		// Obtain image refence
		CCLTRY ( Prepare::extract ( pImg, v, &pImgUse, &pMat ) );

		// First
		if (hr == S_OK && _RCP(First))
			{
			// Find contours likes to crash readily
			try
				{
				// Execute (crashy)
				hr = S_FALSE;
//				cv::findContours (	*(pMat->mat), contours, CV_RETR_EXTERNAL, 
//											CV_CHAIN_APPROX_SIMPLE );

				// Debug
				lprintf ( LOG_INFO, L"Contours size %d\r\n", contours.size() );
				for (int c = 0;c < contours.size();++c)
					lprintf ( LOG_INFO, L"%d) Size %d\r\n", c, contours[c].size() );
				}	// try2
			catch ( cv::Exception & )
				{
				lprintf ( LOG_INFO, L"findContours threw an exception\r\n" );
				hr = S_FALSE;
				}	// catch

			// Reset enumeration
			iIdx = 0;
			}	// if

		// More ?
		CCLTRYE ( iIdx < contours.size(), ERROR_NOT_FOUND );

		// Next
		if (hr == S_OK)
			{
			// Extract

			// For next enumeration
			++iIdx;

			}	// if

		// Result
//		if (hr == S_OK)
//			_EMT(Fire,adtIUnknown(pImgUse));
//		else
//			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pMat);
		_RELEASE(pImgUse);
		}	// if

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

