////////////////////////////////////////////////////////////////////////
//
//									AT.CPP
//
//				Implementation of the pixel access image node.
//
////////////////////////////////////////////////////////////////////////

#include "imagel_.h"
#include <stdio.h>

// Globals

At :: At ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pImg	= NULL;
	}	// At

HRESULT At :: onAttach ( bool bAttach )
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
		if (pnDesc->load ( adtString(L"X"), vL ) == S_OK)
			iX = vL;
		if (pnDesc->load ( adtString(L"Y"), vL ) == S_OK)
			iY = vL;
		pnDesc->load ( adtString(L"Value"), vAt );
		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pImg);
		}	// else

	return hr;
	}	// onAttach

HRESULT At :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

	// Load or store
	if (_RCP(Load) || _RCP(Store))
		{
		IDictionary	*pImgUse = NULL;
		cvMatRef		*pMat		= NULL;
		adtValue		vL;

		// Obtain image refence
		CCLTRY ( Prepare::extract ( pImg, v, &pImgUse, &pMat ) );

		// Process
		if (hr == S_OK)
			{
			// Open CV uses exceptions
			try
				{
				// Unfortunately the type has to be checked to ensure the correct
				// template is used.
	//			dbgprintf ( L"%d , %d\r\n", (S32)iX, (S32) iY );
/*				switch (CV_MAT_DEPTH(pMat->mat->type()))
					{
					// 8-bit
					case CV_8U :
						if (_RCP(Load))
							hr = adtValue::copy ( adtInt(pMat->mat->getMat(cv::ACCESS_READ).at<U8>(iY,iX)), vL );
						else
							pMat->mat->getMat(cv::ACCESS_WRITE).at<U8>(iY,iX) = adtInt(vAt);
						break;
					case CV_8S :
						if (_RCP(Load))
							hr = adtValue::copy ( adtInt(pMat->mat->getMat(cv::ACCESS_READ).at<S8>(iY,iX)), vL );
						else
							pMat->mat->getMat(cv::ACCESS_WRITE).at<S8>(iY,iX) = adtInt(vAt);
						break;

					// 16-bit
					case CV_16U :
						if (_RCP(Load))
							hr = adtValue::copy ( adtInt(pMat->mat->getMat(cv::ACCESS_READ).at<U16>(iY,iX)), vL );
						else
							pMat->mat->getMat(cv::ACCESS_WRITE).at<U16>(iY,iX) = adtInt(vAt);
						break;
					case CV_16S :
						if (_RCP(Load))
							hr = adtValue::copy ( adtInt(pMat->mat->getMat(cv::ACCESS_READ).at<S16>(iY,iX)), vL );
						else
							pMat->mat->getMat(cv::ACCESS_WRITE).at<S16>(iY,iX) = adtInt(vAt);
						break;

					// 32-bit
					case CV_32S :
						if (_RCP(Load))
							hr = adtValue::copy ( adtInt(pMat->mat->getMat(cv::ACCESS_READ).at<S32>(iY,iX)), vL );
						else
							pMat->mat->getMat(cv::ACCESS_WRITE).at<S32>(iY,iX) = adtInt(vAt);
						break;
					case CV_32F :
						if (_RCP(Load))
							hr = adtValue::copy ( adtFloat(pMat->mat->getMat(cv::ACCESS_READ).at<float>(iY,iX)), vL );
						else
							pMat->mat->getMat(cv::ACCESS_WRITE).at<float>(iY,iX) = adtFloat(vAt);
						break;
					}	// switch
*/
				switch (CV_MAT_DEPTH(pMat->mat->type()))
					{
					// 8-bit
					case CV_8U :
						if (_RCP(Load))
							hr = adtValue::copy ( adtInt(pMat->mat->at<U8>(iY,iX)), vL );
						else
							pMat->mat->at<U8>(iY,iX) = adtInt(vAt);
						break;
					case CV_8S :
						if (_RCP(Load))
							hr = adtValue::copy ( adtInt(pMat->mat->at<S8>(iY,iX)), vL );
						else
							pMat->mat->at<S8>(iY,iX) = adtInt(vAt);
						break;

					// 16-bit
					case CV_16U :
						if (_RCP(Load))
							hr = adtValue::copy ( adtInt(pMat->mat->at<U16>(iY,iX)), vL );
						else
							pMat->mat->at<U16>(iY,iX) = adtInt(vAt);
						break;
					case CV_16S :
						if (_RCP(Load))
							hr = adtValue::copy ( adtInt(pMat->mat->at<S16>(iY,iX)), vL );
						else
							pMat->mat->at<S16>(iY,iX) = adtInt(vAt);
						break;

					// 32-bit
					case CV_32S :
						if (_RCP(Load))
							hr = adtValue::copy ( adtInt(pMat->mat->at<S32>(iY,iX)), vL );
						else
							pMat->mat->at<S32>(iY,iX) = adtInt(vAt);
						break;
					case CV_32F :
						if (_RCP(Load))
							hr = adtValue::copy ( adtFloat(pMat->mat->at<float>(iY,iX)), vL );
						else
							pMat->mat->at<float>(iY,iX) = adtFloat(vAt);
						break;
					}	// switch

				}	// try
			catch ( cv::Exception ex )
				{
				lprintf ( LOG_ERR, L"OpenCV Exception" );
				hr = E_UNEXPECTED;
				}	// catch
			}	// if

		// Result
		if (hr == S_OK)
			{
			if (_RCP(Load))
				_EMT(Load,adtIUnknown(pImgUse));
			else
				_EMT(Store,adtIUnknown(pImgUse));
			}	// if

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
	else if (_RCP(X))
		iX = v;
	else if (_RCP(Y))
		iY = v;
	else if (_RCP(Value))
		adtValue::copy ( v, vAt );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

