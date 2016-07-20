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

		// Images for left and right or images for left and scalar for right
		if (	hr == S_OK && 
				adtValue::type(vL) == VTYPE_UNK &&
				!adtValue::empty(vR) )
			{
			IDictionary	*pImgL	= NULL;
			IDictionary	*pImgR	= NULL;
			IDictionary	*pImgO	= NULL;
			cvMatRef		*pMatL	= NULL;
			cvMatRef		*pMatR	= NULL;
			cvMatRef		*pMatO	= NULL;
			bool			bImgR		= false;

			// Image information
			CCLTRY(Prepare::extract ( NULL, vL, &pImgL, &pMatL ));
			if (hr == S_OK && adtValue::type(vR) == VTYPE_UNK)
				{
				CCLTRY(Prepare::extract ( NULL, vR, &pImgR, &pMatR ));
				CCLOK ( bImgR = true; )
				}	// if
			CCLTRY(Prepare::extract ( NULL, v, &pImgO, &pMatO ));

			// Apply operation
			if (hr == S_OK)
				{
				switch (iOp)
					{
					case MATHOP_ADD :
						if (bImgR)
							cv::add ( *(pMatL->mat), *(pMatR->mat), *(pMatO->mat) );
						else
							cv::add ( *(pMatL->mat), cv::Scalar(adtFloat(vR)), *(pMatO->mat) );
						break;
					case MATHOP_SUB :
						if (bImgR)
							cv::subtract ( *(pMatL->mat), *(pMatR->mat), *(pMatO->mat) );
						else
							cv::subtract ( *(pMatL->mat), cv::Scalar(adtFloat(vR)), *(pMatO->mat) );
						break;
					case MATHOP_MUL :
						if (bImgR)
							cv::multiply ( *(pMatL->mat), *(pMatR->mat), *(pMatO->mat) );
						else
							cv::multiply ( *(pMatL->mat), cv::Scalar(adtFloat(vR)), *(pMatO->mat) );
						break;
					case MATHOP_DIV :
						if (bImgR)
							cv::divide ( *(pMatL->mat), *(pMatR->mat), *(pMatO->mat) );
						else
							cv::divide ( *(pMatL->mat), cv::Scalar(adtFloat(vR)), *(pMatO->mat) );
						break;

					// Bitwise
					case MATHOP_AND :
						if (bImgR)
							cv::bitwise_and ( *(pMatL->mat), *(pMatR->mat), *(pMatO->mat) );
						else
							cv::bitwise_and ( *(pMatL->mat), cv::Scalar(adtFloat(vR)), *(pMatO->mat) );
						break;
					case MATHOP_XOR :
						if (bImgR)
							cv::bitwise_xor ( *(pMatL->mat), *(pMatR->mat), *(pMatO->mat) );
						else
							cv::bitwise_xor ( *(pMatL->mat), cv::Scalar(adtFloat(vR)), *(pMatO->mat) );
						break;
					case MATHOP_OR :
						if (bImgR)
							cv::bitwise_or ( *(pMatL->mat), *(pMatR->mat), *(pMatO->mat) );
						else
							cv::bitwise_or ( *(pMatL->mat), cv::Scalar(adtFloat(vR)), *(pMatO->mat) );
						break;

					// Not implemented
					default :
						hr = E_NOTIMPL;
					}	// switch
				}	// if

			// Result
			CCLTRY ( adtValue::copy ( adtIUnknown(pImgO), vRes ) );

			// Clean up
			_RELEASE(pMatO);
			_RELEASE(pImgO);
			_RELEASE(pMatR);
			_RELEASE(pImgR);
			_RELEASE(pMatL);
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

