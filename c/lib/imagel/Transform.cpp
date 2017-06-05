////////////////////////////////////////////////////////////////////////
//
//									TOMOGRAPHY.CPP
//
//				Implementation of the image tomography node.
//
////////////////////////////////////////////////////////////////////////

#include "imagel_.h"
#include <stdio.h>

// Globals

Transform :: Transform ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pImg		= NULL;
	dScl[0]	= dScl[1]	= 1;
	dTrns[0] = dTrns[1]	= 0;
	dRot[0]	= dRot[1]	= 0;
	}	// Transform

HRESULT Transform :: onAttach ( bool bAttach )
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
		if (pnDesc->load ( adtString(L"TranslateX"), vL ) == S_OK)
			dTrns[0] = (vD=vL);
		if (pnDesc->load ( adtString(L"TranslateY"), vL ) == S_OK)
			dTrns[1] = (vD=vL);
		}	// if

	// Detach
	else
		{
		// Clean up
		_RELEASE(pImg);
		}	// else

	return hr;
	}	// onAttach

HRESULT Transform :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Apply transform
	if (_RCP(Fire))
		{
		IDictionary	*pImgUse		= NULL;
		cvMatRef		*pMat			= NULL;
		cv::Point2f	ptSrc[3],ptDst[3];

		// Obtain image refence
		CCLTRY ( Prepare::extract ( pImg, v, &pImgUse, &pMat ) );

		// Transforms (TODO: Rotation,scaling)
		if (hr == S_OK)
			{
			// From point
			ptSrc[0] = ptDst[0] = cv::Point2f(0,0);
			ptSrc[1] = ptDst[1] = cv::Point2f(10,0);
			ptSrc[2] = ptDst[2] = cv::Point2f(0,10);

			// Offset points by specified amount
			ptDst[0].x += (float)dTrns[0];
			ptDst[0].y += (float)dTrns[1];
			ptDst[1].x += (float)dTrns[0];
			ptDst[1].y += (float)dTrns[1];
			ptDst[2].x += (float)dTrns[0];
			ptDst[2].y += (float)dTrns[1];
			}	// if

		// OpenCV uses exceptions
		try
			{
			// Execute
			if (hr == S_OK && pMat->isMat())
				{
				cv::Mat matT,matDst;

				// Transform
				matT = cv::getAffineTransform(ptSrc,ptDst);

				// Apply
				cv::warpAffine ( *(pMat->mat), matDst, matT, pMat->mat->size() );

				// Copy back to source
				matDst.copyTo(*(pMat->mat));
				}	// if
			#ifdef	HAVE_OPENCV_UMAT
			else if (pMat->isUMat())
				{
				cv::UMat matDst;
				cv::Mat	matT;

				// Transform
				matT = cv::getAffineTransform(ptSrc,ptDst);

				// Apply
				cv::warpAffine ( *(pMat->umat), matDst, matT, pMat->umat->size() );

				// Copy back to source
				matDst.copyTo(*(pMat->umat));
				}	// else if
			#endif
			#ifdef	HAVE_OPENCV_CUDA
			else if (pMat->isGPU())
				{
				// TODO: CHECK THIS!
				cv::cuda::GpuMat	matDst;
				cv::Mat				matT;

				// Transform
				matT = cv::getAffineTransform(ptSrc,ptDst);

				// Apply
				cv::warpAffine ( *(pMat->gpumat), matDst, matT, pMat->gpumat->size() );

				// Copy back to source
				matDst.copyTo(*(pMat->gpumat));
				}	// if
			#endif
			}	// try
		catch ( cv::Exception &ex )
			{
			lprintf ( LOG_INFO, L"%S", ex.err.c_str() );
			hr = S_FALSE;
			}	// catch

		// Result
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(pImgUse));
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pMat);
		_RELEASE(pImgUse);
		}	// if

	// Scale/Rotate/Translate
	else if (_RCP(Image))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pImg);
		_QISAFE(unkV,IID_IDictionary,&pImg);
		}	// else if
	else if (_RCP(ScaleX))
		dScl[0] = (vD=v);
	else if (_RCP(ScaleY))
		dScl[1] = (vD=v);
	else if (_RCP(RotateX))
		dRot[0] = (vD=v);
	else if (_RCP(RotateY))
		dRot[1] = (vD=v);
	else if (_RCP(TranslateX))
		dTrns[0] = (vD=v);
	else if (_RCP(TranslateY))
		dTrns[1] = (vD=v);
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive


