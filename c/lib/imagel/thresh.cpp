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
	adtValue::clear(vMin);
	adtValue::clear(vMax);
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
		pnDesc->load ( adtString(L"Minimum"), vMin );
		pnDesc->load ( adtString(L"Maximum"), vMax );
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
		cv::Mat		*pMat		= NULL;
		adtValue		vL;

		// Image to use
		if (pImgUse == NULL)
			{
			adtIUnknown unkV(v);
			CCLTRY(_QISAFE(unkV,IID_IDictionary,&pImgUse));
			}	// if
		else
			pImgUse->AddRef();

		// Wrap CV matrix around image bits
		CCLTRY ( image_to_mat ( pImgUse, &pMat ) );

		// Threshold in any of the specified directions
		if (hr == S_OK && !adtValue::empty(vMin))
			cv::threshold ( *pMat, *pMat, adtDouble(vMin), 0, cv::THRESH_TOZERO );
		if (hr == S_OK && !adtValue::empty(vMax))
			cv::threshold ( *pMat, *pMat, adtDouble(vMax), 0, cv::THRESH_TRUNC );

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
	else if (_RCP(Minimum))
		adtValue::copy ( v, vMin );
	else if (_RCP(Maximum))
		adtValue::copy ( v, vMax );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

