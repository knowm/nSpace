////////////////////////////////////////////////////////////////////////
//
//									ROI.CPP
//
//				Implementation of the image region of interest node.
//
////////////////////////////////////////////////////////////////////////

#include "imagel_.h"
#include <stdio.h>

// Globals

Roi :: Roi ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pSrc	= NULL;
	pDst	= NULL;
	bCopy	= false;
	}	// Roi

HRESULT Roi :: onAttach ( bool bAttach )
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
		if (pnDesc->load ( adtString(L"Left"), vL ) == S_OK)
			iL = vL;
		if (pnDesc->load ( adtString(L"Top"), vL ) == S_OK)
			iT = vL;
		if (pnDesc->load ( adtString(L"Right"), vL ) == S_OK)
			iR = vL;
		if (pnDesc->load ( adtString(L"Bottom"), vL ) == S_OK)
			iB = vL;
		if (pnDesc->load ( adtString(L"Copy"), vL ) == S_OK)
			bCopy = vL;
		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pSrc);
		_RELEASE(pDst);
		}	// else

	return hr;
	}	// onAttach

HRESULT Roi :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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
		IDictionary	*pSrcUse = NULL;
		IDictionary *pDstUse = pDst;
		cvMatRef		*pMatSrc	= NULL;
		cvMatRef		*pMatDst	= NULL;
		cv::Rect		rct;

		// Destination can be provided as part of 'Fire'
		_ADDREF(pDstUse);
		if (hr == S_OK && pDstUse == NULL)
			{
			adtIUnknown		unkV(v);
			_QISAFE(unkV,IID_IDictionary,&pDstUse);
			}	// if

		// State check
		CCLTRYE ( pDstUse != NULL, ERROR_INVALID_STATE );

		// Obtain image refence
		CCLTRY ( Prepare::extract ( pSrc, v, &pSrcUse, &pMatSrc ) );

		// Determine area of Roi.
		if (hr == S_OK)
			{
			// Special case, no dimensions specified means entire image
			if (iL == 0 && iT == 0 && iR == 0 && iB == 0)
				{
				rct.x			= 0;
				rct.y			= 0;
				rct.width	= pMatSrc->cols();
				rct.height	= pMatSrc->rows();
				}	// if
			else
				{
				rct.x			= iL;
				rct.y			= iT;
				rct.width	= iR-iL;
				rct.height	= iB-iT;
				}	// else
			}	// if

		// Open CV uses exceptions
		try
			{
			// Create a new region of interest
			// WARNING: Generation of GPU based ROIs slow ??
			CCLTRYE( (pMatDst = new cvMatRef()) != NULL, E_OUTOFMEMORY );
			if (hr == S_OK && pMatSrc->isGPU())
				{
				// Create ROI
				CCLTRYE((pMatDst->gpumat = new cv::cuda::GpuMat ( *(pMatSrc->gpumat), rct ))
								!= NULL, E_OUTOFMEMORY);

				// Requesting own copy of Roi ?
				if (hr == S_OK && bCopy == true)
					*(pMatDst->gpumat) = pMatDst->gpumat->clone();
				}	// if
			else if (hr == S_OK && pMatSrc->isUMat())
				{
				// Create ROI
				CCLTRYE((pMatDst->umat = new cv::UMat ( *(pMatSrc->umat), rct ))
								!= NULL, E_OUTOFMEMORY);

				// Requesting own copy of Roi ?
				if (hr == S_OK && bCopy == true)
					*(pMatDst->umat) = pMatDst->umat->clone();
				}	// else if
			else
				{
				// Create ROI
				CCLTRYE((pMatDst->mat = new cv::Mat ( *(pMatSrc->mat), rct ))
							!= NULL, E_OUTOFMEMORY);

				// Requesting own copy of Roi ?
				if (hr == S_OK && bCopy == true)
					*(pMatDst->mat) = pMatDst->mat->clone();
				}	// else

			// Result
			CCLTRY ( pDstUse->store (	adtString(L"cvMatRef"), adtIUnknown(pMatDst) ) );
			}	// try
		catch ( cv::Exception ex )
			{
			lprintf ( LOG_ERR, L"OpenCV Exception" );
			hr = E_UNEXPECTED;
			}	// catch

		// Result
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(pDstUse));
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pMatDst);
		_RELEASE(pMatSrc);
		_RELEASE(pDstUse);
		_RELEASE(pSrcUse);
		}	// if

	// State
	else if (_RCP(Source))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pSrc);
		_QISAFE(unkV,IID_IDictionary,&pSrc);
		}	// else if
	else if (_RCP(Destination))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pDst);
		_QISAFE(unkV,IID_IDictionary,&pDst);
		}	// else if
	else if (_RCP(Left))
		iL = v;
	else if (_RCP(Right))
		iR = v;
	else if (_RCP(Top))
		iT = v;
	else if (_RCP(Bottom))
		iB = v;
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

