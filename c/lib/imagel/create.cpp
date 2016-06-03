////////////////////////////////////////////////////////////////////////
//
//									CREATE.CPP
//
//				Implementation of the image creation node.
//
////////////////////////////////////////////////////////////////////////

#include "imagel_.h"
#include <stdio.h>

// Globals

Create :: Create ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pImg		= NULL;
	strFmt	= L"U8X2";
	iW			= 128;
	iH			= 128;
	}	// Create

HRESULT Create :: onAttach ( bool bAttach )
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

		// Defaults
		if (pnDesc->load ( adtString(L"Format"), vL ) == S_OK)
			hr = adtValue::toString ( vL, strFmt );
		if (pnDesc->load ( adtString(L"Width"), vL ) == S_OK)
			iW = vL;
		if (pnDesc->load ( adtString(L"Height"), vL ) == S_OK)
			iH = vL;
		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pImg);
		}	// else

	return hr;
	}	// onAttach

HRESULT Create :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

	// Execute
	if (_RCP(Fire))
		{
		IDictionary	*pImgUse = NULL;
		cvMatRef		*pMat		= NULL;
		U32			cvFmt;

		// Obtain image refence
		CCLTRY ( Prepare::extract ( pImg, v, &pImgUse, NULL ) );

		// Process
		if (hr == S_OK)
			{
			// Map requested format into OpenCV format
			if (!WCASECMP(strFmt,L"F32x2"))
				cvFmt = CV_32FC1;
			else if (!WCASECMP(strFmt,L"U16x2"))
				cvFmt = CV_16UC1;
			else if (!WCASECMP(strFmt,L"S16x2"))
				cvFmt = CV_16SC1;
			else if (!WCASECMP(strFmt,L"U8x2"))
				cvFmt = CV_8UC1;
			else if (!WCASECMP(strFmt,L"S8x2"))
				cvFmt = CV_8SC1;
			else
				hr = E_NOTIMPL;

			// Create a matrix based on the GPU mode
			CCLTRYE( (pMat = new cvMatRef()) != NULL, E_OUTOFMEMORY );
			#if	CV_MAJOR_VERSION == 3
			CCLTRYE ( (pMat->mat = new cv::UMat ( iH, iW, cvFmt )) != NULL,
							E_OUTOFMEMORY );
			#else
			CCLTRYE ( (pMat->mat = new cv::Mat ( iH, iW, cvFmt )) != NULL,
							E_OUTOFMEMORY );
			#endif
			CCLOK   ( *(pMat->mat) = cv::Scalar(0); )

			// Store image in image dictionary
			CCLTRY ( pImgUse->store (	adtString(L"cvMatRef"), 
												adtIUnknown(pMat) ) );
			}	// if

		// Result
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(pImgUse));
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pMat);
		_RELEASE(pImgUse);
		}	// else if

	// State
	else if (_RCP(Image))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pImg);
		_QISAFE(unkV,IID_IDictionary,&pImg);
		}	// else if
	else if (_RCP(Format))
		adtValue::toString ( v, strFmt );
	else if (_RCP(Width))
		iW = v;
	else if (_RCP(Height))
		iH = v;
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

