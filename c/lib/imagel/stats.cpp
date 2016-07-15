////////////////////////////////////////////////////////////////////////
//
//									Stats.CPP
//
//				Implementation of the image statistics node.
//
////////////////////////////////////////////////////////////////////////

#include "imagel_.h"
#include <stdio.h>

// Globals

Stats :: Stats ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pImg	= NULL;
	}	// Stats

HRESULT Stats :: onAttach ( bool bAttach )
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
//		if (pnDesc->load ( adtString(L"Left"), vL ) == S_OK)
//			iL = vL;
		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pImg);
		}	// else

	return hr;
	}	// onAttach

HRESULT Stats :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

		// Limits
		if (hr == S_OK)
			{
			cv::Point	ptMin,ptMax;
			double		dMin,dMax;

			// Values and locations of min and max
			cv::minMaxLoc ( *(pMat->mat), &dMin, &dMax, &ptMin, &ptMax );

			// Result
			CCLTRY ( pImgUse->store ( adtString(L"Min"), adtDouble(dMin) ) );
			CCLTRY ( pImgUse->store ( adtString(L"MinX"), adtInt(ptMin.x) ) );
			CCLTRY ( pImgUse->store ( adtString(L"MinY"), adtInt(ptMin.y) ) );
			CCLTRY ( pImgUse->store ( adtString(L"Max"), adtDouble(dMax) ) );
			CCLTRY ( pImgUse->store ( adtString(L"MaxX"), adtInt(ptMax.x) ) );
			CCLTRY ( pImgUse->store ( adtString(L"MaxY"), adtInt(ptMax.y) ) );
			}	// if

		// Mean, standard deviation
		if (hr == S_OK)
			{
			cv::Scalar 
			mean = cv::mean ( (*pMat->mat) );

			// Result
			CCLTRY ( pImgUse->store ( adtString(L"Mean"), adtDouble(mean[0]) ) );
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

	// Histogram
	else if (_RCP(Histogram))
		{
		IDictionary	*pImgUse = NULL;
		IDictionary	*pImgHst = NULL;
		cvMatRef		*pMat		= NULL;
		cvMatRef		*pMatHst	= NULL;

		// Default to 0-256 graylevels.  Future expansion can have additional options.
		float				range[]		= { 0, 256 };
		const float *	histRange	= { range };
		int				histSize		= 256;

		// Obtain image refence and result image
		CCLTRY ( Prepare::extract ( pImg, v, &pImgUse, &pMat ) );
		CCLTRY ( Prepare::extract ( NULL, v, &pImgHst, &pMatHst ) );

		// Calculate histogram.  Currently defaults to grayscale.
		CCLOK ( cv::calcHist ( pMat->mat, 1, 0, cv::Mat(), *(pMatHst->mat), 1, 
										&histSize, &histRange ); )

		// Result
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(pImgHst));
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pMat);
		_RELEASE(pImg);
		_RELEASE(pMatHst);
		_RELEASE(pImgHst);
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

