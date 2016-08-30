////////////////////////////////////////////////////////////////////////
//
//									CONVERT.CPP
//
//				Implementation of the image format conversion node.
//
////////////////////////////////////////////////////////////////////////

#include "imagel_.h"
#include <stdio.h>

// Globals

Convert :: Convert ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pImg = NULL;
	}	// Convert

HRESULT Convert :: convertTo (	cvMatRef *pMatSrc, cvMatRef *pMatDst, 
											U32 fmt )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Global utility to convert data to a particular format.
	//
	//	PARAMETERS
	//		-	pMatSrc is the source data
	//		-	pMatDst will receive the converted data
	//		-	fmt is the new OpenCV format
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Special cases
	if (pMatSrc->isGPU() && pMatDst->isGPU())
		hr = convertTo ( pMatSrc->gpumat, pMatDst->gpumat, fmt );
	else if (pMatSrc->isUMat() && pMatDst->isUMat())
		{
		// Direct in-memory conversion
		pMatSrc->umat->convertTo ( *(pMatDst->umat), fmt );
		}	// if

	else
		{
		cv::Mat	matCnv;

		// Copy to CPU
		if (pMatSrc->isGPU())
			pMatSrc->gpumat->download ( matCnv );
		else if (pMatSrc->isUMat())
			pMatSrc->umat->copyTo ( matCnv );
		else
			matCnv = *(pMatSrc->mat);

		// Convert 'to' specified format.
		matCnv.convertTo ( matCnv, fmt );

		// Copy back
		if (pMatDst->isGPU())
			pMatDst->gpumat->upload ( matCnv );
		else if (pMatDst->isUMat())
			matCnv.copyTo ( *(pMatDst->umat) );
		else
			matCnv.copyTo ( *(pMatDst->mat) );
		}	// else

	return hr;
	}	// convertTo

HRESULT Convert :: convertTo (	cv::cuda::GpuMat *pMatSrc, 
											cv::cuda::GpuMat *pMatDst, U32 fmt )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Global utility to convert data to a particular format.
	//
	//	PARAMETERS
	//		-	pMatSrc is the source data
	//		-	pMatDst will receive the converted data
	//		-	fmt is the new OpenCV format
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT				hr = S_OK;
	cv::cuda::GpuMat	gpuCnv;

	// Cannot convert into the same array, use intermediate array

	// Convert to temporary location
	pMatSrc->convertTo ( gpuCnv, fmt );

	// Copy to destination
	gpuCnv.copyTo ( *pMatDst );

	return hr;
	}	// convertTo

HRESULT Convert :: onAttach ( bool bAttach )
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
		if (pnDesc->load ( adtString(L"Format"), vL ) == S_OK)
			hr = adtValue::toString ( vL, strTo );
		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pImg);
		}	// else

	return hr;
	}	// onAttach

HRESULT Convert :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

		// Convert 'to' specified format. Add formats as needed.
//		CCLOK ( image_to_debug ( pMat, L"Convert", L"c:/temp/convert1.png" ); )
		if (hr == S_OK)
			{
			if (!WCASECMP(strTo,L"F32x2"))
				convertTo ( pMat, pMat, CV_32FC1 );
			else if (!WCASECMP(strTo,L"U16x2"))
				convertTo ( pMat, pMat, CV_16UC1 );
			else if (!WCASECMP(strTo,L"S16x2"))
				convertTo ( pMat, pMat, CV_16SC1 );
			else if (!WCASECMP(strTo,L"U8x2"))
				convertTo ( pMat, pMat, CV_8UC1 );
			else if (!WCASECMP(strTo,L"S8x2"))
				convertTo ( pMat,pMat,  CV_8SC1 );
			else
				hr = E_NOTIMPL;
			}	// if
//		CCLOK ( image_to_debug ( pMat, L"Convert", L"c:/temp/convert2.png" ); )

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
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

