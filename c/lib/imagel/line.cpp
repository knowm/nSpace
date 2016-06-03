////////////////////////////////////////////////////////////////////////
//
//									LINE.CPP
//
//				Implementation of the line draw node.
//
////////////////////////////////////////////////////////////////////////

#include "imagel_.h"
#include <stdio.h>

// Globals

Line :: Line ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pImg	= NULL;
	iR = iB = iG = 0;
	iX0 = iY0 = iX1 = iY1 = 0;
	iThick = 1;
	}	// Line

HRESULT Line :: onAttach ( bool bAttach )
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
		if (pnDesc->load ( adtString(L"Red"), vL ) == S_OK)
			iR = vL;
		if (pnDesc->load ( adtString(L"Green"), vL ) == S_OK)
			iG = vL;
		if (pnDesc->load ( adtString(L"Blue"), vL ) == S_OK)
			iB = vL;
		if (pnDesc->load ( adtString(L"Thickness"), vL ) == S_OK)
			iThick = vL;
		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pImg);
		}	// else

	return hr;
	}	// onAttach

HRESULT Line :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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
		CCLOK ( cv::line ( *(pMat->mat), cv::Point(iX0,iY0), cv::Point(iX1,iY1), CV_RGB(iR,iG,iB), iThick ); )

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
	else if (_RCP(X0))
		iX0 = v;
	else if (_RCP(X1))
		iX1 = v;
	else if (_RCP(Y0))
		iY0 = v;
	else if (_RCP(Y1))
		iY1 = v;
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

