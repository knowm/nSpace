////////////////////////////////////////////////////////////////////////
//
//									FEATURES.CPP
//
//				Implementation of the image features node.
//
////////////////////////////////////////////////////////////////////////

#include "imagel_.h"
#include <stdio.h>

// Globals

Features :: Features ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pImg			= NULL;

	// Defaults
	strType	= L"";
	}	// Features

HRESULT Features :: onAttach ( bool bAttach )
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
			adtValue::toString ( vL, strType );
		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pImg);
		}	// else

	return hr;
	}	// onAttach

HRESULT Features :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

		// Perform Featuresing operation
		if (hr == S_OK)
			{
			// UMat/OpenCL
			if (pMat->isUMat())
				{
				if (!WCASECMP(L"HoughCircles",strType))
					{
					cv::UMat	matCircles;

					// Perform detection
					cv::HoughCircles ( *(pMat->umat), matCircles, CV_HOUGH_GRADIENT, 1,
												10, 200, 100, 0, 0 );

					// Replace original array
					matCircles.copyTo(*(pMat->umat));
					}	// if
				else if (!WCASECMP(strType,L"Canny"))
					{
					cv::UMat matDst;

					// Execute
					cv::Canny ( *(pMat->umat), matDst, 100, 200 ); 

					// Result
					if (matDst.rows > 0 && matDst.cols > 0)
						matDst.copyTo ( *(pMat->umat) );
					}	// else if
				else
					hr = E_NOTIMPL;
				}	// if

			// GPU
			#ifdef	WITH_CUDA
			else if (pMat->isGPU())
				{
				if (!WCASECMP(L"HoughCircles",strType))
					{
					cv::Mat	matNoGpu;
					cv::Mat	matCircles;

					// Currently no GPU based version
					pMat->gpumat->download ( matNoGpu );

					// Perform detection
					cv::HoughCircles ( matNoGpu, matCircles, CV_HOUGH_GRADIENT, 1,
												10, 200, 100, 0, 0 );

					// Replace original array
					pMat->gpumat->upload ( matCircles );
					}	// if
				else if (!WCASECMP(strType,L"Canny"))
					{
/*
					// TODO: Current calling any function on 'pgpuCanny' crashes.
					// For now perform CPU based edge detection.
					cv::Mat	cpuMat,cpuEdges;

					// Download into CPU memory
					pMat->gpumat->download ( cpuMat );

					// Perform detection
					cv::Canny ( cpuMat, cpuEdges, 100, 200 ); 

					// Result
					pMat->gpumat->upload ( cpuEdges );
*/
					cv::cuda::GpuMat	edges;

					// Create canny detector algorithm if necessary
					if (pgpuCanny == NULL)
						pgpuCanny = cv::cuda::createCannyEdgeDetector ( 100, 200 );

					// Perform detection
					if (pgpuCanny != NULL)
						pgpuCanny->detect ( *(pMat->gpumat), edges );

					// Result
					edges.copyTo ( *(pMat->gpumat) );
					}	// else if
				else
					hr = E_NOTIMPL;
				}	// if
			#endif

			// CPU
			else
				{
				if (!WCASECMP(L"HoughCircles",strType))
					{
					std::vector<cv::Vec3f>	circles;

					// Perform detection
					cv::HoughCircles ( *(pMat->mat), circles, CV_HOUGH_GRADIENT, 
												2, 5, 100, 20, 0, 0 );

					// Debug
//if (circles.size()>1)
//	DebugBreak();
					#ifdef _DEBUG
					for (int i = 0;i < circles.size();++i)
						lprintf ( LOG_DBG, L"Circle %g,%g,%g\r\n",
										(float)circles[i][0], 
										(float)circles[i][1], 
										(float)circles[i][2] );
					#endif

					hr = S_OK;

					// Replace original array
//					matCircles.copyTo(*(pMat->mat));
					}	// if
				else if (!WCASECMP(strType,L"Canny"))
					{
					cv::Mat matDst;

					// Execute
					cv::Canny ( *(pMat->mat), matDst, 100, 200 ); 

					// Result
					if (matDst.rows > 0 && matDst.cols > 0)
						matDst.copyTo ( *(pMat->mat) );
					}	// else if
				else
					hr = E_NOTIMPL;
				}	// else

			}	// if

		// Debug
		if (hr != S_OK)
			lprintf ( LOG_DBG, L"'%s' failed : 0x%x\r\n", (LPCWSTR)strType, hr );

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

