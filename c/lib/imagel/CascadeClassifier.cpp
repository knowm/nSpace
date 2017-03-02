////////////////////////////////////////////////////////////////////////
//
//									NORMAL.CPP
//
//				Implementation of the image normalization node.
//
////////////////////////////////////////////////////////////////////////

#include "imagel_.h"
#include <stdio.h>

// Globals

CascadeClassifier :: CascadeClassifier ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pImg	= NULL;
	bLoad	= false;
	}	// CascadeClassifier

HRESULT CascadeClassifier :: onAttach ( bool bAttach )
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
		if (pnDesc->load ( adtString(L"Location"), vL ) == S_OK)
			adtValue::toString ( vL, strLoc );
		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pImg);
		}	// else

	return hr;
	}	// onAttach

HRESULT CascadeClassifier :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Perform detection
	if (_RCP(Fire))
		{
		IDictionary	*pImgUse = NULL;
		cvMatRef		*pMat		= NULL;

		// State check
		CCLTRYE ( bLoad == true || strLoc.length() > 0, ERROR_INVALID_STATE );

		// Obtain image refence
		CCLTRY ( Prepare::extract ( pImg, v, &pImgUse, &pMat ) );

		// OpenCV uses exceptions
		try
			{
			// Does the classifier need to be loaded ?
			if (!bLoad)
				{
				char *paLoc = NULL;

				// Attempt load
				CCLTRY( strLoc.toAscii(&paLoc) );
				CCLOK	( cc.load ( paLoc ); )

				// Clean up
				_FREEMEM(paLoc);
				}	// if

			// Perform detection
			if (pMat->isUMat())
				{
				}	// if
			#ifdef	WITH_CUDA
			else if (pMat->isGPU())
				{
				}	// else if
			#endif
			else if (pMat->isMat())
				{
				std::vector<cv::Rect_<int>>	objs;
				cc.detectMultiScale ( *(pMat->mat), objs );
				lprintf ( LOG_INFO, L"Count : %d", objs.size() );
				}	// else if
			}	// try
		catch ( cv::Exception &ex )
			{
			lprintf ( LOG_WARN, L"%S\r\n", ex.err.c_str() );
			hr = E_UNEXPECTED;
			}	// catch

		// Result
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(pImgUse));
		else
			_EMT(Error,adtInt(hr));

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
	else if (_RCP(Location))
		hr = adtValue::toString ( v, strLoc );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

