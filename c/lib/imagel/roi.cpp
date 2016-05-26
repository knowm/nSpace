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

HRESULT Roi :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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
		IDictionary	*pSrcUse = pSrc;
		cv::Mat		*pMatSrc	= NULL;
		cv::Mat		*pMatDst	= NULL;
		adtValue		vL;

		// State check
		CCLTRYE ( pDst != NULL, ERROR_INVALID_STATE );

		// Image to use
		if (pSrcUse == NULL)
			{
			adtIUnknown unkV(v);
			CCLTRY(_QISAFE(unkV,IID_IDictionary,&pSrcUse));
			}	// if
		else
			pSrcUse->AddRef();

		// Ensure destination is clear
		if (hr == S_OK && pDst->load ( adtString(L"cv::Mat"), vL ) == S_OK)
			{
			// Specified matrix
			CCLTRYE( (pMatDst = (cv::Mat *)(U64)adtLong(vL)) != NULL,
						ERROR_INVALID_STATE );

			// Clean up
			if (pMatDst != NULL)
				{
				delete pMatDst;
				pMatDst = NULL;
				}	// if
			pDst->remove ( adtString(L"cv::Mat") );
			}	// if

		// Image must be 'uploaded'
		CCLTRY ( pSrcUse->load (	adtString(L"cv::Mat"), vL ) );
		CCLTRYE( (pMatSrc = (cv::Mat *)(U64)adtLong(vL)) != NULL,
					ERROR_INVALID_STATE );

		// Adjust Roi.  Could error out but be flexible.
		// Set limits so there is at least one pixel in each direction.
		if (hr == S_OK)
			{
			// Width
			if			(iL < 0)							iL = 0;
			else if	((int)iL >= pMatSrc->cols)	iL = pMatSrc->cols-1;
			if			(iR <= 0)						iR = 1;
			else if	((int)iR >= pMatSrc->cols)	iR = pMatSrc->cols;
			// Height
			if			(iT < 0)							iT = 0;
			else if	((int)iT >= pMatSrc->rows)	iT = pMatSrc->rows-1;
			if			(iB <= 0)						iB = 1;
			else if	((int)iB >= pMatSrc->rows)	iB = pMatSrc->rows;
			}	// if

		// Open CV uses exceptions
		try
			{
			// Create a new region of interest
			CCLTRYE((pMatDst = new cv::Mat ( *pMatSrc, cv::Rect(iL,iT,(iR-iL),(iB-iT)) ))
							!= NULL, E_OUTOFMEMORY);
			CCLTRY ( pDst->store (	adtString(L"cv::Mat"), adtLong((U64)pMatDst) ) );
			}	// try
		catch ( cv::Exception ex )
			{
			lprintf ( LOG_ERR, L"OpenCV Exception" );
			hr = E_UNEXPECTED;
			}	// catch

		// Result
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(pDst));
		else
			_EMT(Error,adtInt(hr));

		// Clean up
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

