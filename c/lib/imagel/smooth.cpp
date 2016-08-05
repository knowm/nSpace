////////////////////////////////////////////////////////////////////////
//
//									SMOOTH.CPP
//
//				Implementation of the image smoothing node.
//
////////////////////////////////////////////////////////////////////////

#include "imagel_.h"
#include <stdio.h>

// Globals

Smooth :: Smooth ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pImg	= NULL;

	// Defaults
	strType	= L"MedianBlur";
	iSz		= 3;
	}	// Smooth

HRESULT Smooth :: onAttach ( bool bAttach )
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
		if (pnDesc->load ( adtString(L"Size"), vL ) == S_OK)
			iSz = vL;
		if (pnDesc->load ( adtString(L"Type"), vL ) == S_OK)
			adtValue::toString ( vL, strType );
		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pImg);
		}	// else

	return hr;
	}	// onAttach

HRESULT Smooth :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

		// Obtain image refence
		CCLTRY ( Prepare::extract ( pImg, v, &pImgUse, &pMat ) );

		// Perform smoothing operation
		if (hr == S_OK)
			{
			if (pMat->isGPU())
				{
				cv::Mat		matNoGpu;

				// There must be a GPU way of doing this.  Manually convolution with kernel ?
				pMat->gpumat->download ( matNoGpu );

				// Execute
				if (!strType.length())
					cv::blur ( matNoGpu, matNoGpu, cv::Size(iSz,iSz) );
				else if (!WCASECMP(L"Median",strType))
					cv::medianBlur ( matNoGpu, matNoGpu, iSz );

				// Restore
				pMat->gpumat->upload ( matNoGpu );
				}	// if
			else if (pMat->isUMat())
				{
				if (!strType.length())
					cv::blur ( *(pMat->umat), *(pMat->umat), cv::Size(iSz,iSz) );
				else if (!WCASECMP(L"Median",strType))
					cv::medianBlur ( *(pMat->umat), *(pMat->umat), iSz );
				}	// if
			else
				{
				if (!strType.length())
					cv::blur ( *(pMat->mat), *(pMat->mat), cv::Size(iSz,iSz) );
				else if (!WCASECMP(L"Median",strType))
					cv::medianBlur ( *(pMat->mat), *(pMat->mat), iSz );
				}	// else
			}	// if

		// Debug
		if (hr != S_OK)
			lprintf ( LOG_DBG, L"Blur '%s' failed : 0x%x\r\n", (LPCWSTR)strType, hr );

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
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

