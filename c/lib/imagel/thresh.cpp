////////////////////////////////////////////////////////////////////////
//
//									THRESH.CPP
//
//				Implementation of the image threshold node.
//
////////////////////////////////////////////////////////////////////////

#include "imagel_.h"
#include <stdio.h>

// Globals

Threshold :: Threshold ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pImg	= NULL;
	vT		= 0;
	strOp	= L"Zero";
	}	// Threshold

HRESULT Threshold :: onAttach ( bool bAttach )
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
		if (pnDesc->load ( adtString(L"Op"), vL ) == S_OK)
			adtValue::toString ( vL, strOp );
		pnDesc->load ( adtString(L"Value"), vT );
		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pImg);
		}	// else

	return hr;
	}	// onAttach

HRESULT Threshold :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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
		IDictionary	*pImgUse = pImg;
		cvMatRef		*pMat		= NULL;
		adtValue		vL;

		// Image to use
		if (pImgUse == NULL)
			{
			adtIUnknown unkV(v);
			CCLTRY(_QISAFE(unkV,IID_IDictionary,&pImgUse));
			}	// if
		else
			pImgUse->AddRef();

		// DEBUG
//		double dMin,dMax;
//		cv::minMaxIdx ( *pMat, &dMin, &dMax );
//		dbgprintf ( L"dMin %g dMax %g\r\n", dMin, dMax );

		// Image must be 'uploaded'
		CCLTRY ( pImgUse->load (	adtString(L"cvMatRef"), vL ) );
		CCLTRYE( (pMat = (cvMatRef *)(U64)adtLong(vL)) != NULL,
					ERROR_INVALID_STATE );

		// Perform operation
		if (hr == S_OK)
			{
			if (!WCASECMP(strOp,L"Zero"))
				cv::threshold ( *(pMat->mat), *(pMat->mat), adtDouble(vT), 0, cv::THRESH_TOZERO );
			else if (!WCASECMP(strOp,L"Truncate"))
				cv::threshold ( *(pMat->mat), *(pMat->mat), adtDouble(vT), 0, cv::THRESH_TRUNC );
			}	// if

		// Result
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(pImgUse));
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pImgUse);
		}	// if

	// State
	else if (_RCP(Image))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pImg);
		_QISAFE(unkV,IID_IDictionary,&pImg);
		}	// else if
	else if (_RCP(Value))
		adtValue::copy ( v, vT );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

