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

Normalize :: Normalize ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pImg	= NULL;

	// Defaults
	adtValue::copy ( adtInt(0), vFrom );
	adtValue::copy ( adtInt(255), vTo );
	}	// Normalize

HRESULT Normalize :: onAttach ( bool bAttach )
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
		if (pnDesc->load ( adtString(L"From"), vL ) == S_OK)
			adtValue::copy ( vL, vFrom );
		if (pnDesc->load ( adtString(L"To"), vL ) == S_OK)
			adtValue::copy ( vL, vTo );
		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pImg);
		}	// else

	return hr;
	}	// onAttach

HRESULT Normalize :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

		// Perform operation
		if (hr == S_OK)
			{
			if (pMat->isMat())
				cv::normalize ( *(pMat->mat), *(pMat->mat), adtDouble(vFrom), adtDouble(vTo), cv::NORM_MINMAX );
			#ifdef	HAVE_OPENCV_UMAT
			else if (pMat->isUMat())
				cv::normalize ( *(pMat->umat), *(pMat->umat), adtDouble(vFrom), adtDouble(vTo), cv::NORM_MINMAX );
			#endif
			#ifdef	HAVE_OPENCV_CUDA
			else if (pMat->isGPU())
				cv::cuda::normalize ( *(pMat->gpumat), *(pMat->gpumat), adtDouble(vFrom), adtDouble(vTo), 
												cv::NORM_MINMAX, -1 );
			#endif
			}	// if

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
	else if (_RCP(From))
		hr = adtValue::copy ( v, vFrom );
	else if (_RCP(To))
		hr = adtValue::copy ( v, vTo );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

