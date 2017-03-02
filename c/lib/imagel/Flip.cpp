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

HRESULT Flip :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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
		IDictionary	*pImgUse = NULL;
		cvMatRef		*pMat		= NULL;

		// Obtain image refence
		CCLTRY ( Prepare::extract ( pImg, v, &pImgUse, &pMat ) );

		// Perform operation
//		CCLOK ( image_to_debug ( pMat, L"Flip", L"c:/temp/flip1.png" ); )
		if (hr == S_OK && (bHorz || bVert))
			{
			if (pMat->isUMat())
				cv::flip ( *(pMat->umat), *(pMat->umat),	(bHorz && bVert)	? -1 : 
																		(bVert)				? 0 : 1 );
			#ifdef	WITH_CUDA
			else if (pMat->isGPU())
				{
				cv::cuda::GpuMat	matFlip;

				// NOTE NOTE NOTE
				// Apparently there are some operations that do not work on the GPU
				// where src == dst so copy is required.
				pMat->gpumat->copyTo ( matFlip );

				// Perform flip
				cv::cuda::flip ( matFlip, *(pMat->gpumat),	
										(bHorz && bVert)	? -1 : 
										(bVert)				? 0 : 1 );
				}	// if
			#endif
			else
				cv::flip ( *(pMat->mat), *(pMat->mat),	(bHorz && bVert)	? -1 : 
																	(bVert)				? 0 : 1 );
			}	// if
//		CCLOK ( image_to_debug ( pMat, L"Flip", L"c:/temp/flip2.png" ); )

		// Result
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(pImgUse));
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pMat);
		_RELEASE(pImgUse);
		}	// if

	// Transpose (another node ?)
	else if (_RCP(Transpose))
		{
		IDictionary	*pImgUse = NULL;
		cvMatRef		*pMat		= NULL;

		// Obtain image refence
		CCLTRY ( Prepare::extract ( pImg, v, &pImgUse, &pMat ) );

		// Perform operation
//		CCLOK ( image_to_debug ( pMat, L"Transpose", L"c:/temp/trans1.png" ); )
		if (hr == S_OK)
			{
			if (pMat->isUMat())
				{
				cv::UMat	matTrans;

				// NOTE: UMat does not like source and destination to be the same
				pMat->umat->copyTo ( matTrans );
				cv::transpose ( matTrans, *(pMat->umat) );
				}	// else if
			#ifdef	WITH_CUDA
			else if (pMat->isGPU())
				cv::cuda::transpose ( *(pMat->gpumat), *(pMat->gpumat) );
			#endif
			else
				{
				cv::Mat	matT;

				// NOTE: Mat does not like source and destination to be the same
				// Example: Rotating something WidthX2700 gave 2800xWidth, size not the same
				pMat->mat->copyTo ( matT );
				cv::transpose ( matT, *(pMat->mat) );
				}	// else
			}	// if
//		CCLOK ( image_to_debug ( pMat, L"Transpose", L"c:/temp/trans2.png" ); )

		// Result
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(pImgUse));
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pMat);
		_RELEASE(pImgUse);
		}	// else if

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

