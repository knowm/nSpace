////////////////////////////////////////////////////////////////////////
//
//									GRADIENT.CPP
//
//				Implementation of the image gradient node.
//
////////////////////////////////////////////////////////////////////////

#include "imagel_.h"
#include <stdio.h>

// Globals

Gradient :: Gradient ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pImg			= NULL;
	strType		= L"Laplacian";
	iDx			= 1;
	iDy			= 1;
	iSzk			= 3;
	#ifdef	HAVE_OPENCV_CUDA
	pgpuSobel	= NULL;
	pgpuScharr	= NULL;
	pgpuLaplace	= NULL;
	#endif
	}	// Gradient

HRESULT Gradient :: onAttach ( bool bAttach )
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
		if (pnDesc->load ( adtString(L"Type"), vL ) == S_OK)
			hr = adtValue::toString ( vL, strType );
		if (pnDesc->load ( adtString(L"dX"), vL ) == S_OK)
			iDx = vL;
		if (pnDesc->load ( adtString(L"dY"), vL ) == S_OK)
			iDy = vL;
		if (pnDesc->load ( adtString(L"Size"), vL ) == S_OK)
			iSzk = vL;
		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pImg);
		#ifdef	HAVE_OPENCV_CUDA
		_DELETE(pgpuSobel);
		_DELETE(pgpuScharr);
		_DELETE(pgpuLaplace);
		#endif
		}	// else

	return hr;
	}	// onAttach

HRESULT Gradient :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

		// Execute
		if (hr == S_OK)
			{
			// Find contours likes to crash readily
			try
				{
				// Execute 
				if (pMat->isMat())
					{
					// Gradients want own destination
					cv::Mat matDst;
 
					// Perform requested type
					if (!WCASECMP(strType,L"Laplacian"))
						cv::Laplacian ( *(pMat->mat), matDst, CV_16SC1, iSzk );
					else if (!WCASECMP(strType,L"Sobel"))
						cv::Sobel ( *(pMat->mat), matDst, CV_16SC1, iDx, iDy, iSzk );
					else if (!WCASECMP(strType,L"Scharr"))
						cv::Sobel ( *(pMat->mat), matDst, CV_16SC1, iDx, iDy, CV_SCHARR );
					else
						hr = E_NOTIMPL;

					// Result
					if (matDst.rows > 0 && matDst.cols > 0)
						matDst.copyTo ( *(pMat->mat) );
					}	// else
				#ifdef	HAVE_OPENCV_UMAT
				else if (pMat->isUMat())
					{
					// Perform requested type
					if (!WCASECMP(strType,L"Laplacian"))
						cv::Laplacian ( *(pMat->umat), *(pMat->umat), CV_16SC1, iSzk );
					else if (!WCASECMP(strType,L"Sobel"))
						cv::Sobel ( *(pMat->umat), *(pMat->umat), CV_16SC1, iDx, iDy, iSzk );
					else if (!WCASECMP(strType,L"Scharr"))
						cv::Sobel ( *(pMat->umat), *(pMat->umat), CV_16SC1, iDx, iDy, -1 );
					else
						hr = E_NOTIMPL;
					}	// else if
				#endif
				#ifdef	HAVE_OPENCV_CUDA
				else if (pMat->isGPU())
					{
					// Perform requested type
					if (!WCASECMP(strType,L"Laplacian"))
						{
						if (pgpuLaplace == NULL)
							pgpuLaplace = cv::cuda::createLaplacianFilter ( pMat->gpumat->type(),
												CV_16SC1, iSzk );
						if (pgpuLaplace != NULL)
							pgpuLaplace->apply ( *(pMat->gpumat), *(pMat->gpumat) );
						}	// if
					else if (!WCASECMP(strType,L"Sobel"))
						{
						if (pgpuSobel == NULL)
							pgpuSobel = cv::cuda::createSobelFilter ( pMat->gpumat->type(),
												CV_16SC1, iDx, iDy, iSzk );
						if (pgpuSobel != NULL)
							pgpuSobel->apply ( *(pMat->gpumat), *(pMat->gpumat) );
						}	// if
					else if (!WCASECMP(strType,L"Scharr"))
						{
						if (pgpuScharr == NULL)
							pgpuScharr = cv::cuda::createScharrFilter ( pMat->gpumat->type(),
												CV_16SC1, iDx, iDy, iSzk );
						if (pgpuScharr != NULL)
							pgpuScharr->apply ( *(pMat->gpumat), *(pMat->gpumat) );
						}	// if
					else
						hr = E_NOTIMPL;
					}	// if
				#endif

				}	// try
			catch ( cv::Exception &ex )
				{
				lprintf ( LOG_INFO, L"gradient '%s' threw an exception:%S", 
								(LPCWSTR)strType, ex.err.c_str() );
				hr = S_FALSE;
				}	// catch

			}	// if

		// Result
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(pImgUse));
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
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

