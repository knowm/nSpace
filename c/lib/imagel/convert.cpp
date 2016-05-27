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

HRESULT Convert :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

		// Image to use.  Previously specified or passed in.
		if (pImgUse == NULL)
			{
			adtIUnknown unkV(v);
			CCLTRY(_QISAFE(unkV,IID_IDictionary,&pImgUse));
			}	// if
		else
			pImgUse->AddRef();

		// Image must be 'uploaded'
		CCLTRY ( pImgUse->load (	adtString(L"cvMatRef"), vL ) );
		CCLTRYE( (pMat = (cvMatRef *)(U64)adtLong(vL)) != NULL,
					ERROR_INVALID_STATE );

		// Convert 'to' specified format. Add formats as needed.
		if (!WCASECMP(strTo,L"F32x2"))
			pMat->mat->convertTo ( *(pMat->mat), CV_32FC1 );
		else if (!WCASECMP(strTo,L"U16x2"))
			pMat->mat->convertTo ( *(pMat->mat), CV_16UC1 );
		else if (!WCASECMP(strTo,L"S16x2"))
			pMat->mat->convertTo ( *(pMat->mat), CV_16SC1 );
		else if (!WCASECMP(strTo,L"U8x2"))
			pMat->mat->convertTo ( *(pMat->mat), CV_8UC1 );
		else if (!WCASECMP(strTo,L"S8x2"))
			pMat->mat->convertTo ( *(pMat->mat), CV_8SC1 );
		else
			hr = E_NOTIMPL;

		// Result
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(pImgUse));
		else
			_EMT(Error,adtInt(hr));

		// Clean up
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

