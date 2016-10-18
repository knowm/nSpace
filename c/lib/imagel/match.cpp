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
	pImg		= NULL;
	pTmp		= NULL;
	tmType	= 0;
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
		// Clean up
//		if (pTm != NULL)
//			{
//			delete pTm;
//			pTm = NULL;
//			}	// if
		_RELEASE(pImg);
		_RELEASE(pTmp);
		}	// else

	return hr;
	}	// onAttach

HRESULT Match :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

		// All images must be of the same type
		CCLTRYE (	(pMatT->mat != NULL && pMatR->mat != NULL) ||
						(pMatT->umat != NULL && pMatR->umat != NULL) ||
						(pMatT->gpumat != NULL && pMatR->gpumat != NULL),
						ERROR_INVALID_STATE );

		// Create a matrix to receive the results
		CCLTRY ( Create::create ( pImgO, pMatR->rows() - pMatT->rows() + 1,
													pMatR->cols() - pMatT->cols() + 1,
													CV_32FC1, &pMatO, (pMatT->mat != NULL) ) );

		// Perform matching with normalized coefficients
try
	{
		if (hr == S_OK)
			{
			cv::Mat	matR,matT,matC;

//image_to_debug ( pMatR, L"Match", L"c:/temp/MatchR.png" );
//image_to_debug ( pMatT, L"Match", L"c:/temp/MatchT.png" );

			//
			// NOTE: Currently getting different results from all different methods,
			// CPU, CUDA and OpenCL (Umat).  Until this is figured out, downshift into CPU mode.
			//
			if (pMatR->isGPU())
				{
				pMatR->gpumat->download(matR);
				pMatT->gpumat->download(matT);
				}	// if
			else if (pMatR->isUMat())
				{
				matR = pMatR->umat->getMat(cv::ACCESS_READ);
				matT = pMatT->umat->getMat(cv::ACCESS_READ);
				}	// else if
			else
				{
				matR = *(pMatR->mat);
				matT = *(pMatT->mat);
				}	// else

			// Perform match
			cv::matchTemplate ( matR, matT, matC, CV_TM_CCORR_NORMED );

			// Copy result to destination
			if (pMatO->isGPU())
				pMatO->gpumat->upload(matC);
			else if (pMatR->isUMat())
				matC.copyTo ( *(pMatO->umat) );
			else
				matC.copyTo ( *(pMatO->mat) );

//image_to_debug ( pMatO, L"Match", L"c:/temp/MatchC1.png" );

			/*
			if (pMatR->isGPU())
				{
				try
					{
					// Create a matching template
					if (pTm == NULL || (pMatR->gpumat->type() != tmType))
						{
						CCLTRYE ( (pTm = cv::cuda::createTemplateMatching ( pMatR->gpumat->type(), 
										CV_TM_CCORR_NORMED )) != NULL, E_OUTOFMEMORY );
						CCLOK ( tmType = pMatR->gpumat->type(); )
						}	// if

					// Execute
					CCLOK ( pTm->match ( *(pMatR->gpumat), *(pMatT->gpumat), *(pMatO->gpumat) ); )
					}	// try
				catch ( cv::Exception &ex )
					{
					lprintf ( LOG_WARN, L"createTemplateMatching exception:%S\r\n",
									ex.err.c_str() );
					hr = E_UNEXPECTED;
					}	// catch
				}	// if
			else if (pMatR->isUMat())
				{
				cv::UMat matC;

				cv::matchTemplate ( *(pMatR->umat), *(pMatT->umat), matC,
//				cv::matchTemplate ( *(pMatR->umat), *(pMatT->umat), *(pMatO->umat),
											CV_TM_CCORR_NORMED );

dbgprintf ( L"%d %d %d %d %d %d\r\n", 
				pMatR->umat->type(), pMatR->umat->channels(), pMatR->umat->depth(),
				pMatR->umat->dims, pMatR->umat->elemSize(), (pMatR->umat->type() & CV_MAT_DEPTH_MASK) );

dbgprintf ( L"%d %d %d %d %d %d\r\n", 
				matC.type(), matC.channels(), matC.depth(),
				matC.dims, matC.elemSize(), (matC.type() & CV_MAT_DEPTH_MASK) );

				matC.copyTo ( *(pMatO->umat) );
				}	// else if
			else
				{
				cv::UMat matC;

//				cv::matchTemplate ( *(pMatR->mat), *(pMatT->mat), *(pMatO->mat),
				cv::matchTemplate ( *(pMatR->mat), *(pMatT->mat), matC,
											CV_TM_CCORR_NORMED );

dbgprintf ( L"%d %d %d %d %d %d\r\n", 
				pMatR->mat->type(), pMatR->mat->channels(), pMatR->mat->depth(),
				pMatR->mat->dims, pMatR->mat->elemSize(), (pMatR->mat->type() & CV_MAT_DEPTH_MASK) );

dbgprintf ( L"%d %d %d %d %d %d\r\n", 
				matC.type(), matC.channels(), matC.depth(),
				matC.dims, matC.elemSize(), (matC.type() & CV_MAT_DEPTH_MASK) );

				matC.copyTo ( *(pMatO->mat) );
				}	// else
			*/
			}	// if

			}	// try
		catch ( cv::Exception &e )
			{
			lprintf ( LOG_INFO, L"%S\r\n", e.err.c_str() );
			hr = E_UNEXPECTED;
			}	// catch

		// Clean up
		_RELEASE(pMatO);
		_RELEASE(pImgO);
		_RELEASE(pMatT);
		_RELEASE(pImgT);
		_RELEASE(pMatR);
		_RELEASE(pImgR);

		// Debug
		if (hr != S_OK)
			lprintf ( LOG_DBG, L"matchTemplate failed : 0x%x\r\n", hr );

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

