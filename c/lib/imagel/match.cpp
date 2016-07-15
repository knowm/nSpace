////////////////////////////////////////////////////////////////////////
//
//									MATCH.CPP
//
//				Implementation of the template matching image node.
//
////////////////////////////////////////////////////////////////////////

#include "imagel_.h"
#include <stdio.h>

// Globals

Match :: Match ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pImg = NULL;
	pTmp = NULL;
	}	// Match

HRESULT Match :: onAttach ( bool bAttach )
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
//		pnDesc->load ( adtString(L"Left"), vL );
		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pImg);
		_RELEASE(pTmp);
		}	// else

	return hr;
	}	// onAttach

HRESULT Match :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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
		IDictionary	*pImgR	= NULL;
		IDictionary	*pImgT	= NULL;
		IDictionary	*pImgO	= NULL;
		cvMatRef		*pMatR	= NULL;
		cvMatRef		*pMatT	= NULL;
		cvMatRef		*pMatO	= NULL;
		adtIUnknown	unkV(v);

		// State check
		CCLTRYE( pImg != NULL && pTmp != NULL, ERROR_INVALID_STATE );
		CCLTRY ( _QISAFE(unkV,IID_IDictionary,&pImgO) );

		// Image information
		CCLTRY(Prepare::extract ( pImg, adtValue(), &pImgR, &pMatR ));
		CCLTRY(Prepare::extract ( pTmp, adtValue(), &pImgT, &pMatT ));

		// Create an matrix to receive the results
		CCLTRY ( Create::create ( pImgO, (pMatR->mat->rows) - (pMatT->mat->rows) + 1,
													(pMatR->mat->cols) - (pMatT->mat->cols) + 1,
													CV_32FC1, &pMatO ) );

		// Perform matching with normalized coefficients
		CCLOK ( cv::matchTemplate ( *(pMatR->mat), *(pMatT->mat), *(pMatO->mat),
												CV_TM_CCORR_NORMED ); )

		// Clean up
		_RELEASE(pMatO);
		_RELEASE(pImgO);
		_RELEASE(pMatT);
		_RELEASE(pImgT);
		_RELEASE(pMatR);
		_RELEASE(pImgR);

		// Result
		if (hr == S_OK)
			_EMT(Fire,v);
		else
			{
//			lprintf ( LOG_ERR, L"%s:Fire:Error:hr 0x%x:%d\r\n", (LPCWSTR)strnName, hr, iOp );
			_EMT(Error,adtInt(hr) );
			}	// else
		}	// else if

	// State
	else if (_RCP(Image))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pImg);
		_QISAFE(unkV,IID_IDictionary,&pImg);
		}	// else if
	else if (_RCP(Template))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pTmp);
		_QISAFE(unkV,IID_IDictionary,&pTmp);
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

