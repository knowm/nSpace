////////////////////////////////////////////////////////////////////////
//
//									RESIZE.CPP
//
//				Implementation of the image resize node.
//
////////////////////////////////////////////////////////////////////////

#include "imagel_.h"
#include <stdio.h>

// Globals

Resize :: Resize ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pImg		= NULL;
	iW			= 128;
	iH			= 128;
	}	// Resize

HRESULT Resize :: onAttach ( bool bAttach )
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

HRESULT Resize :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

		// State check
		CCLTRYE ( iW > 0 && iH > 0, ERROR_INVALID_STATE );

		// Obtain image refence
		CCLTRY ( Prepare::extract ( pImg, v, &pImgUse, &pMat ) );

		// Resize the image
		if (hr == S_OK)
			{
			if (pMat->isMat())
				cv::resize ( *(pMat->mat), *(pMat->mat), cv::Size(iW,iH) );
			#ifdef	HAVE_OPENCV_UMAT
			else if (pMat->isUMat())
				cv::resize ( *(pMat->umat), *(pMat->umat), cv::Size(iW,iH) );
			#endif
			#ifdef	HAVE_OPENCV_CUDA
			else if (pMat->isGPU())
				cv::cuda::resize ( *(pMat->gpumat), *(pMat->gpumat), cv::Size(iW,iH) );
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
		}	// else if

	// State
	else if (_RCP(Image))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pImg);
		_QISAFE(unkV,IID_IDictionary,&pImg);
		}	// else if
	else if (_RCP(Width))
		iW = v;
	else if (_RCP(Height))
		iH = v;
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

