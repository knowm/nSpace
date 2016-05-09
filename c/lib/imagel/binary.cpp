////////////////////////////////////////////////////////////////////////
//
//									BINARY.CPP
//
//				Implementation of the binary operation image node.
//
////////////////////////////////////////////////////////////////////////

#include "imagel_.h"
#include <stdio.h>

// Globals

Binary :: Binary ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	iOp = MATHOP_ADD;
	adtValue::clear(vL);
	adtValue::clear(vR);
	}	// Binary

HRESULT Binary :: onAttach ( bool bAttach )
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
		pnDesc->load ( adtString(L"Left"), vL );
		pnDesc->load ( adtString(L"Rightt"), vR );
		if (	pnDesc->load ( adtStringSt(L"Op"), vL ) == S_OK	&& 
				adtValue::type(vL) == VTYPE_STR						&&
				vL.pstr != NULL )
			mathOp ( vL.pstr, &iOp );
		}	// if

	// Detach
	else
		{
		// Shutdown
		}	// else

	return hr;
	}	// onAttach

HRESULT Binary :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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
		// State check
		CCLTRYE ( adtValue::empty(vL) == false, ERROR_INVALID_STATE );
		CCLTRYE ( adtValue::empty(vR) == false, ERROR_INVALID_STATE );
		CCLTRYE ( iOp >= MATHOP_ADD, ERROR_INVALID_STATE );

		// Incoming value will receive result.
		// TODO: Seperate receptor for this ?
		CCLTRYE ( adtValue::type(v) == VTYPE_UNK, ERROR_INVALID_STATE );

		// Images for left and right
		if (	hr == S_OK && 
				adtValue::type(vL) == VTYPE_UNK &&
				adtValue::type(vR) == VTYPE_UNK )
			{
			IDictionary	*pImgL	= NULL;
			IDictionary	*pImgR	= NULL;
			IDictionary	*pImgO	= NULL;
			cv::Mat		*pMatL	= NULL;
			cv::Mat		*pMatR	= NULL;
			cv::Mat		*pMatO	= NULL;

			// Image dictionaries
			CCLTRY(_QISAFE(adtIUnknown(vL),IID_IDictionary,&pImgL));
			CCLTRY(_QISAFE(adtIUnknown(vR),IID_IDictionary,&pImgR));
			CCLTRY(_QISAFE(adtIUnknown(v),IID_IDictionary,&pImgO));

			// Images must be 'uploaded'
			CCLTRY ( pImgL->load (	adtString(L"cv::Mat"), vL ) );
			CCLTRYE( (pMatL = (cv::Mat *)(U64)adtLong(vL)) != NULL,
						ERROR_INVALID_STATE );
			CCLTRY ( pImgR->load (	adtString(L"cv::Mat"), vL ) );
			CCLTRYE( (pMatR = (cv::Mat *)(U64)adtLong(vL)) != NULL,
						ERROR_INVALID_STATE );
			CCLTRY ( pImgO->load (	adtString(L"cv::Mat"), vL ) );
			CCLTRYE( (pMatO = (cv::Mat *)(U64)adtLong(vL)) != NULL,
						ERROR_INVALID_STATE );

			// Apply operation
			switch (iOp)
				{
				case MATHOP_ADD :
					cv::add ( *pMatL, *pMatR, *pMatO );
					break;
				case MATHOP_SUB :
					cv::subtract ( *pMatL, *pMatR, *pMatO );
					break;
				case MATHOP_MUL :
					cv::multiply ( *pMatL, *pMatR, *pMatO );
					break;
				case MATHOP_DIV :
					cv::divide ( *pMatL, *pMatR, *pMatO );
					break;

				// Not implemented
				default :
					hr = E_NOTIMPL;
				}	// switch

			// Result
			CCLTRY ( adtValue::copy ( adtIUnknown(pImgO), vRes ) );

			// Clean up
			_RELEASE(pImgO);
			_RELEASE(pImgR);
			_RELEASE(pImgL);
			}	// if

		// Not handled/error
		else
			hr = E_NOTIMPL;

		// Result
		if (hr == S_OK)
			_EMT(Fire,vRes);
		else
			{
//			lprintf ( LOG_ERR, L"%s:Fire:Error:hr 0x%x:%d\r\n", (LPCWSTR)strnName, hr, iOp );
			_EMT(Error,adtInt(hr) );
			}	// else
		}	// else if

	// State
	else if (_RCP(Left))
		adtValue::copy ( v, vL );
	else if (_RCP(Right))
		adtValue::copy ( v, vR );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

