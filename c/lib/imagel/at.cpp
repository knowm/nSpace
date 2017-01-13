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

HRESULT At :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Load
	if (_RCP(Load))
		{
		IDictionary	*pImgUse = NULL;
		IList			*pvLst	= NULL;
		cvMatRef		*pMat		= NULL;
		bool			bSingle	= false;
		S32			x			= 0;
		S32			y			= 0;
		adtValue		vLd;

		// Value to use
		const ADTVALUE *pvUse	= (!adtValue::empty(vAt)) ? &vAt : &v;

		// Obtain image refence
		CCLTRY ( Prepare::extract ( pImg, v, &pImgUse, &pMat ) );

		// Valid range ?
		CCLTRYE (	(iX >= 0 && (S32)iX < pMat->cols()) &&
						(iY >= 0 && (S32)iY < pMat->rows()), ERROR_INVALID_STATE );

		// Starting coordinates
		CCLOK ( x = iX; )
		CCLOK ( y = iY; )

		//
		// Determine if specified value is a list
		//
		if (hr == S_OK)
			{
			adtIUnknown		unkV(*pvUse);

			// Assume a single value
			bSingle = true;

			// Object ?
			if ((IUnknown *)(NULL) != unkV)
				{
				// Not a single value
				bSingle = false;

				// List ?
				if (	_QI(unkV,IID_IList,&pvLst) != S_OK )
					hr = ERROR_NOT_SUPPORTED;
				}	// if

			}	// if

		// Load value or values from image
		while (hr == S_OK && (bSingle || y < pMat->rows()))
			{
			// Open CV uses exceptions
			try
				{
				// GPU
				if (pMat->isGPU())
					{
					// Anything can be done ?
					static bool bFirst = true;
					if (bFirst)
						{
						lprintf ( LOG_ERR, L"Per pixel access for GPU images not available" );
						bFirst = false;
						}	// if

					// Unsupported since image is in GPU memory
					hr = E_NOTIMPL;
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
							hr = adtValue::copy ( adtInt(matAt.at<U8>(y,x)), vLd );
							break;
						case CV_8S :
							hr = adtValue::copy ( adtInt(matAt.at<S8>(y,x)), vLd );
							break;

						// 16-bit
						case CV_16U :
							hr = adtValue::copy ( adtInt(matAt.at<U16>(y,x)), vLd );
							break;
						case CV_16S :
							hr = adtValue::copy ( adtInt(matAt.at<S16>(y,x)), vLd );
							break;

						// 32-bit
						case CV_32S :
							hr = adtValue::copy ( adtInt(matAt.at<S32>(y,x)), vLd );
							break;
						case CV_32F :
							hr = adtValue::copy ( adtFloat(matAt.at<float>(y,x)), vLd );
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
							hr = adtValue::copy ( adtInt(pMat->mat->at<U8>(y,x)), vLd );
							break;
						case CV_8S :
							hr = adtValue::copy ( adtInt(pMat->mat->at<S8>(y,x)), vLd );
							break;

						// 16-bit
						case CV_16U :
							hr = adtValue::copy ( adtInt(pMat->mat->at<U16>(y,x)), vLd );
							break;
						case CV_16S :
							hr = adtValue::copy ( adtInt(pMat->mat->at<S16>(y,x)), vLd );
							break;

						// 32-bit
						case CV_32S :
							hr = adtValue::copy ( adtInt(pMat->mat->at<S32>(y,x)), vLd );
							break;
						case CV_32F :
							hr = adtValue::copy ( adtFloat(pMat->mat->at<float>(y,x)), vLd );
							break;
						}	// switch
					}	// else

				}	// try
			catch ( cv::Exception ex )
				{
				lprintf ( LOG_ERR, L"OpenCV Exception" );
				hr = E_UNEXPECTED;
				}	// catch

			// Single value done or proceed to next value
			if (bSingle)
				bSingle = false;
			else
				{
				pvLst->write(vLd);
				++y;
				}	// else
			}	// while

		// Result
		if (hr == S_OK)
			_EMT(Load,(pvLst == NULL) ? vLd : adtIUnknown(pvLst) );
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pvLst);
		_RELEASE(pMat);
		_RELEASE(pImgUse);
		}	// if

	// Store
	else if (_RCP(Store))
		{
		IDictionary	*pImgUse = NULL;
		IDictionary	*pvDct	= NULL;
		IContainer	*pvCnt	= NULL;
		cvMatRef		*pMat		= NULL;
		IIt			*pIt		= NULL;
		bool			bSingle	= false;
		S32			x			= 0;
		S32			y			= 0;
		adtValue		vSt;

		// Value to use
		const ADTVALUE *pvUse	= (!adtValue::empty(vAt)) ? &vAt : &v;

		// Obtain image refence
		CCLTRY ( Prepare::extract ( pImg, v, &pImgUse, &pMat ) );

		//
		// Determine if specified value is a dictionary, list or single value
		//
		if (hr == S_OK)
			{
			adtIUnknown		unkV(*pvUse);

			// Assume not a single value
			bSingle = false;

			// Object ?
			if ((IUnknown *)(NULL) != unkV)
				{
				// Dictionary/List ?
				if (	_QI(unkV,IID_IDictionary,&pvDct) == S_OK )
					{
					// Iterate keys/values
					CCLTRY ( pvDct->iterate(&pIt) );

					// Assign as container
					CCLOK ( pvCnt = pvDct; )
					_ADDREF(pvCnt);
					}	// if
				else if (_QI(unkV,IID_IContainer,&pvCnt) == S_OK )
					hr = pvCnt->iterate(&pIt);
				else
					hr = ERROR_NOT_SUPPORTED;
				}	// if

			// Value
			else
				{
				hr			= adtValue::copy ( *pvUse, vSt );
				bSingle	= true;
				}	// else

			}	// if

		//
		// If value is an object and a target coordinate is -1,
		// a list of values is being appended.
		// TODO: Support appending column
		//
		if (hr == S_OK && pvCnt != NULL && iY == -1)
			{
			// TODO: Currently only CPU based matrices supported 
			CCLTRYE ( pMat->isMat() == true, ERROR_NOT_SUPPORTED );

			// Increase by one row
			if (hr == S_OK)
				{
				U32	sz;

				// Check if number of columns in image needs to be increased
				CCLTRY ( pvCnt->size(&sz) );
				if (hr == S_OK && (S32)sz > pMat->cols())
					{
					// Columns to append
					cv::Mat	matCols(pMat->rows(),(sz-pMat->cols()),pMat->mat->type());

					// Append
					cv::hconcat ( *(pMat->mat), matCols, *(pMat->mat) );
					}	// if

				// Append new row
				if (hr == S_OK)
					{
					// Row to append
					cv::Mat	matRow(1,pMat->cols(),pMat->mat->type());

					// Append
					cv::vconcat ( *(pMat->mat), matRow, *(pMat->mat) );
					}	// if

				}	// if

			// Starting coordinates
			x = 0;
			y = (pMat->rows()-1);
			}	// if

		// Otherwise coordinates should be valid
		else
			{
			// Valid range ?
			CCLTRYE (	(iX >= 0 && (S32)iX < pMat->cols()) &&
							(iY >= 0 && (S32)iY < pMat->rows()), ERROR_INVALID_STATE );

			// Starting coordinates
			x = iX;
			y = iY;
			}	// else

		// Store value or values into array
		while (hr == S_OK && (bSingle || pIt->read(vSt) == S_OK))
			{
			// Open CV uses exceptions
			try
				{
				// GPU
				if (pMat->isGPU())
					{
					// Anything can be done ?
					static bool bFirst = true;
					if (bFirst)
						{
						lprintf ( LOG_ERR, L"Per pxel access for GPU images not available" );
						bFirst = false;
						}	// if

					// Unsupported since image is in GPU memory
					hr = E_NOTIMPL;
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
							matAt.at<U8>(y,x) = adtInt(vSt);
							break;
						case CV_8S :
							matAt.at<S8>(y,x) = adtInt(vSt);
							break;

						// 16-bit
						case CV_16U :
							matAt.at<U16>(y,x) = adtInt(vSt);
							break;
						case CV_16S :
							matAt.at<S16>(y,x) = adtInt(vSt);
							break;

						// 32-bit
						case CV_32S :
							matAt.at<S32>(y,x) = adtInt(vSt);
							break;
						case CV_32F :
							matAt.at<float>(y,x) = adtFloat(vSt);
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
							pMat->mat->at<U8>(y,x) = adtInt(vSt);
							break;
						case CV_8S :
							pMat->mat->at<S8>(y,x) = adtInt(vSt);
							break;

						// 16-bit
						case CV_16U :
							pMat->mat->at<U16>(y,x) = adtInt(vSt);
							break;
						case CV_16S :
							pMat->mat->at<S16>(y,x) = adtInt(vSt);
							break;

						// 32-bit
						case CV_32S :
							pMat->mat->at<S32>(y,x) = adtInt(vSt);
							break;
						case CV_32F :
							pMat->mat->at<float>(y,x) = adtFloat(vSt);
							break;
						}	// switch
					}	// else

				}	// try
			catch ( cv::Exception ex )
				{
				lprintf ( LOG_ERR, L"OpenCV Exception" );
				hr = E_UNEXPECTED;
				}	// catch

			// Single value done or proceed to next value
			if (bSingle)
				bSingle = false;
			else
				{
				pIt->next();
				++x;
				}	// else
			}	// while

		// Result
		if (hr == S_OK)
			_EMT(Store,adtIUnknown(pImgUse));
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pIt);
		_RELEASE(pvDct);
		_RELEASE(pvCnt);
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

