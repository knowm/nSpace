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
				// GPU
				if (pMat->isGPU())
					{
					const uchar *ptrcRow = NULL;
					uchar			*ptrRow	= NULL;

					// Access row ptr.
					if (_RCP(Load))
						ptrcRow = pMat->gpumat->ptr(iY);
					else
						ptrRow = pMat->gpumat->ptr(iY);

					// Access array
					switch (CV_MAT_DEPTH(pMat->gpumat->type()))
						{
						// 8-bit
						case CV_8U :
							if (_RCP(Load))
								hr = adtValue::copy ( adtInt(ptrcRow[iX]), vL );
							else
								*ptrRow = (U8) adtInt(vAt);
							break;
						case CV_8S :
							if (_RCP(Load))
								hr = adtValue::copy ( adtInt((S8)ptrcRow[iX]), vL );
							else
								*ptrRow = ((S8) adtInt(vAt));
							break;

						// 16-bit
						case CV_16U :
							if (_RCP(Load))
								hr = adtValue::copy ( adtInt(((U16 *)ptrcRow)[iX]), vL );
							else
								((U16 *)ptrRow)[iX] = (U16) adtInt(vAt);
							break;
						case CV_16S :
							if (_RCP(Load))
								hr = adtValue::copy ( adtInt(((S16 *)ptrcRow)[iX]), vL );
							else
								((S16 *)ptrRow)[iX] = (S16) adtInt(vAt);
							break;

						// 32-bit
						case CV_32S :
							if (_RCP(Load))
								hr = adtValue::copy ( adtInt(((S32 *)ptrcRow)[iX]), vL );
							else
								((S32 *)ptrRow)[iX] = (S32) adtInt(vAt);
							break;
						case CV_32F :
							if (_RCP(Load))
								hr = adtValue::copy ( adtFloat(((float *)ptrcRow)[iX]), vL );
							else
								((float *)ptrRow)[iX] = (float) adtFloat(vAt);
							break;
						}	// switch
					}	// if

				// UMAT
				else if (pMat->isUMat())
					{
					// Access array
					cv::Mat	matAt = pMat->umat->getMat( (_RCP(Load)) ? cv::ACCESS_READ : cv::ACCESS_WRITE );
					switch (CV_MAT_DEPTH(matAt.type()))
						{
						// 8-bit
						case CV_8U :
							if (_RCP(Load))
								hr = adtValue::copy ( adtInt(matAt.at<U8>(iY,iX)), vL );
							else
								matAt.at<U8>(iY,iX) = adtInt(vAt);
							break;
						case CV_8S :
							if (_RCP(Load))
								hr = adtValue::copy ( adtInt(matAt.at<S8>(iY,iX)), vL );
							else
								matAt.at<S8>(iY,iX) = adtInt(vAt);
							break;

						// 16-bit
						case CV_16U :
							if (_RCP(Load))
								hr = adtValue::copy ( adtInt(matAt.at<U16>(iY,iX)), vL );
							else
								matAt.at<U16>(iY,iX) = adtInt(vAt);
							break;
						case CV_16S :
							if (_RCP(Load))
								hr = adtValue::copy ( adtInt(matAt.at<S16>(iY,iX)), vL );
							else
								matAt.at<S16>(iY,iX) = adtInt(vAt);
							break;

						// 32-bit
						case CV_32S :
							if (_RCP(Load))
								hr = adtValue::copy ( adtInt(matAt.at<S32>(iY,iX)), vL );
							else
								matAt.at<S32>(iY,iX) = adtInt(vAt);
							break;
						case CV_32F :
							if (_RCP(Load))
								hr = adtValue::copy ( adtFloat(matAt.at<float>(iY,iX)), vL );
							else
								matAt.at<float>(iY,iX) = adtFloat(vAt);
							break;
						}	// switch
					}	// else if

				// CPU
				else
					{
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
					}	// else

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

