////////////////////////////////////////////////////////////////////////
//
//									FLIP.CPP
//
//				Implementation of the image flip node.
//
////////////////////////////////////////////////////////////////////////

#include "imagel_.h"
#include <stdio.h>

// Globals

Flip :: Flip ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pImg	= NULL;
	bHorz = false;
	bVert = false;
	}	// Flip

HRESULT Flip :: onAttach ( bool bAttach )
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
		if (pnDesc->load ( adtString(L"Vertical"), vL ) == S_OK)
			bVert = vL;
		if (pnDesc->load ( adtString(L"Horizontal"), vL ) == S_OK)
			bHorz = vL;
		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pImg);
		}	// else

	return hr;
	}	// onAttach

HRESULT Flip :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

		// Image must be 'uploaded'
		CCLTRY ( pImgUse->load (	adtString(L"cvMatRef"), vL ) );
		CCLTRYE( (pMat = (cvMatRef *)(U64)adtLong(vL)) != NULL,
					ERROR_INVALID_STATE );

		// Perform operation
		if (hr == S_OK && (bHorz || bVert))
			cv::flip ( *(pMat->mat), *(pMat->mat),	(bHorz && bVert)	? -1 : 
																(bVert)				? 0 : 1 );

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
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

