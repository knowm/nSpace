////////////////////////////////////////////////////////////////////////
//
//									Stats.CPP
//
//				Implementation of the image statistics node.
//
////////////////////////////////////////////////////////////////////////

#include "imagel_.h"
#include <stdio.h>

// Globals

Stats :: Stats ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pImg			= NULL;
	bEnt			= false;
	bBoundRct	= false;
	bSum			= false;
	}	// Stats

HRESULT Stats :: onAttach ( bool bAttach )
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
		if (pnDesc->load ( adtString(L"Entropy"), vL ) == S_OK)
			bEnt = vL;
		if (pnDesc->load ( adtString(L"BoundRect"), vL ) == S_OK)
			bBoundRct = vL;
		if (pnDesc->load ( adtString(L"NonZero"), vL ) == S_OK)
			bNonZero = vL;
		if (pnDesc->load ( adtString(L"Sum"), vL ) == S_OK)
			bSum = vL;
		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pImg);
		}	// else

	return hr;
	}	// onAttach

HRESULT Stats :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

		// OpenCV throws exceptions
		try
			{
			// Limits
			if (hr == S_OK)
				{
				cv::Point	ptMin,ptMax;
				double		dMin,dMax;
				S32			w,h;

				// Values and locations of min and max
				if (pMat->isMat())
					{
					cv::minMaxLoc ( *(pMat->mat), &dMin, &dMax, &ptMin, &ptMax );
					w = pMat->mat->cols;
					h = pMat->mat->rows;
					}	// if
				#ifdef	HAVE_OPENCV_UMAT
				else if (pMat->isUMat())
					{
					cv::minMaxLoc ( *(pMat->umat), &dMin, &dMax, &ptMin, &ptMax );
					w = pMat->umat->cols;
					h = pMat->umat->rows;
					}	// if
				#endif
				#ifdef	HAVE_OPENCV_CUDA
				else if (pMat->isGPU())
					{
					cv::cuda::minMaxLoc ( *(pMat->gpumat), &dMin, &dMax, &ptMin, &ptMax );
					w = pMat->gpumat->cols;
					h = pMat->gpumat->rows;
					}	// if
				#endif

				// Result
				CCLTRY ( pImgUse->store ( adtString(L"Width"), adtInt(w) ) );
				CCLTRY ( pImgUse->store ( adtString(L"Height"), adtInt(h) ) );
				CCLTRY ( pImgUse->store ( adtString(L"Min"), adtDouble(dMin) ) );
				CCLTRY ( pImgUse->store ( adtString(L"MinX"), adtInt(ptMin.x) ) );
				CCLTRY ( pImgUse->store ( adtString(L"MinY"), adtInt(ptMin.y) ) );
				CCLTRY ( pImgUse->store ( adtString(L"Max"), adtDouble(dMax) ) );
				CCLTRY ( pImgUse->store ( adtString(L"MaxX"), adtInt(ptMax.x) ) );
				CCLTRY ( pImgUse->store ( adtString(L"MaxY"), adtInt(ptMax.y) ) );
				}	// if

			// Mean, standard deviation
			if (hr == S_OK)
				{
				cv::Scalar mean	= 0;
				cv::Scalar stddev = 0;

				// Perform calculation
				if (pMat->isMat())
					cv::meanStdDev ( *(pMat->mat), mean, stddev );
				#ifdef	HAVE_OPENCV_UMAT
				else if (pMat->isUMat())
					cv::meanStdDev ( *(pMat->umat), mean, stddev );
				#endif
				#ifdef	HAVE_OPENCV_CUDA
				else if (pMat->isGPU())
					cv::cuda::meanStdDev ( *(pMat->gpumat), mean, stddev );
				#endif

				// Debug
	//			if (mean[0] > 1000 || mean[0] < -1000)
	//				dbgprintf ( L"Hi\r\n" );

				// Result
				CCLTRY ( pImgUse->store ( adtString(L"Mean"), adtDouble(mean[0]) ) );
				CCLTRY ( pImgUse->store ( adtString(L"StdDev"), adtDouble(stddev[0]) ) );
				}	// if

			// Entropy calculation
			if (hr == S_OK && bEnt == true && pMat->channels() == 1)
				{
				cv::Mat				matHst,matLog;

				// Default to 0-256 graylevels.  Future expansion can have additional options.
				float				range[]		= { 0, 256 };
				const float *	histRange	= { range };
				int				histSize		= 256;
				float				ent			= 0.0f;

				// Calculate the historgram of the image
				if (pMat->isMat())
					{
					cv::Mat	matHst,matLog;

					// Histogram
					cv::calcHist ( pMat->mat, 1, 0, cv::noArray(), matHst, 1, 
													&histSize, &histRange );

					// Normalize
					matHst /= (double)pMat->mat->total();

					// Ensure no log of zeroes
					cv::add ( matHst, cv::Scalar::all(1e-20), matHst );

					// Compute entropy
					cv::log ( matHst, matLog );
					cv::multiply(matHst,matLog,matHst);
					ent = (float)(-1*cv::sum(matHst).val[0]);
					}	// else
				#ifdef	HAVE_OPENCV_UMAT
				else if (pMat->isUMat())
					{
					cv::Mat	matHst,matLog;

					// TODO: not working yet, keeps using same bits

					// 'calcHist' not available for UMat ?
					matHst = pMat->umat->getMat(cv::ACCESS_READ);

					// Histogram
					cv::calcHist ( &matHst, 1, 0, cv::noArray(), 
										matHst, 1, &histSize, &histRange );

					// Normalize
					matHst /= (double)pMat->umat->total();

					// Ensure no log of zeroes
					cv::add ( matHst, cv::Scalar::all(1e-20), matHst );

					// Compute entropy
					cv::log ( matHst, matLog );
					cv::multiply(matHst,matLog,matHst);
					ent = (float)(-1*cv::sum(matHst).val[0]);
					}	// if
				#endif
				#ifdef	HAVE_OPENCV_CUDA
				else if (pMat->isGPU())
					{
					cv::cuda::GpuMat	gpuHst,gpuLog;

					// Histogram
					cv::cuda::calcHist ( *(pMat->gpumat), gpuHst );

					// Normalize
					cv::cuda::divide ( gpuHst, cv::Scalar(pMat->gpumat->rows*pMat->gpumat->cols), gpuHst );

					// Ensure no log of zeroes
					cv::cuda::add ( gpuHst, cv::Scalar::all(1e-20), gpuHst );

					// Compute entropy
					cv::cuda::log ( gpuHst, gpuLog );
					cv::cuda::multiply(gpuHst,gpuLog,gpuHst);
					ent = (float)(-1*cv::cuda::sum(gpuHst).val[0]);
					}	// if
				#endif

				// Result
				CCLTRY ( pImgUse->store ( adtString(L"Entropy"), adtDouble(ent) ) );
				}	// if

			// Bounding rectangle
			if (hr == S_OK && bBoundRct == true)
				{
				cv::Rect rct;

				// Calculate bounding rectangle, assumes array of points
				if (pMat->isMat())
					rct = cv::boundingRect ( *(pMat->mat) );
				#ifdef	HAVE_OPENCV_UMAT
				else if (pMat->isUMat())
					rct = cv::boundingRect ( *(pMat->umat) );
				#endif
				#ifdef	HAVE_OPENCV_CUDA
				else if (pMat->isGPU())
					{
					cv::Mat		matNoGpu;

					// Currently no GPU based version ?
					pMat->gpumat->download ( matNoGpu );

					// Execute
					rct = cv::boundingRect ( matNoGpu );
					}	// if
				#endif

				// Result
				CCLTRY ( pImgUse->store ( adtString(L"Left"), adtInt(rct.x) ) );
				CCLTRY ( pImgUse->store ( adtString(L"Top"), adtInt(rct.y) ) );
				CCLTRY ( pImgUse->store ( adtString(L"Width"), adtInt(rct.width) ) );
				CCLTRY ( pImgUse->store ( adtString(L"Height"), adtInt(rct.height) ) );
				}	// if

			// Pixel sum
			if (hr == S_OK && bSum == true)
				{
				cv::Scalar	sum = 0;

				// Calculate total sum of pixels
				if (pMat->isMat())
					sum = cv::sum(*(pMat->mat));
				#ifdef	HAVE_OPENCV_UMAT
				else if (pMat->isUMat())
					sum = cv::sum(*(pMat->umat));
				#endif
				#ifdef	HAVE_OPENCV_CUDA
				else if (pMat->isGPU())
					sum = cv::cuda::sum(*(pMat->gpumat));
				#endif

				// Result
				CCLTRY ( pImgUse->store ( adtString(L"Sum"), adtDouble(sum[0]) ) );
				}	// if

			// Non-zero pixels
			if (hr == S_OK && bNonZero == true)
				{
				int nZ = 0;

				// Perform calculation
				if (pMat->isMat())
					nZ = cv::countNonZero ( *(pMat->mat) );
				#ifdef	HAVE_OPENCV_UMAT
				else if (pMat->isUMat())
					nZ = cv::countNonZero ( *(pMat->umat) );
				#endif
				#ifdef	HAVE_OPENCV_CUDA
				else if (pMat->isGPU())
					nZ = cv::cuda::countNonZero ( *(pMat->gpumat) );
				#endif

				// Result
				CCLTRY ( pImgUse->store ( adtString(L"NonZero"), adtInt(nZ) ) );
				}	 // if

			}	// try
		catch ( cv::Exception &e )
			{
			lprintf ( LOG_INFO, L"%S\r\n", e.err.c_str() );
			hr = E_UNEXPECTED;
			}	// catch

		// Debug
		if (hr != S_OK)
			lprintf ( LOG_DBG, L"stats failed : 0x%x\r\n", hr );

		// Result
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(pImgUse));
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pMat);
		_RELEASE(pImgUse);
		}	// if

	// Histogram
	else if (_RCP(Histogram))
		{
		IDictionary	*pImgUse = NULL;
		IDictionary	*pImgHst = NULL;
		cvMatRef		*pMat		= NULL;
		cvMatRef		*pMatHst	= NULL;

		// Default to 0-256 graylevels.  Future expansion can have additional options.
		float				range[]		= { 0, 256 };
		const float *	histRange	= { range };
		int				histSize		= 256;

		// Obtain image refence and result image
		CCLTRY ( Prepare::extract ( pImg, v, &pImgUse, &pMat ) );
		CCLTRY ( Prepare::extract ( NULL, v, &pImgHst, &pMatHst ) );

		// Calculate the historgram of the image
		if (hr == S_OK)
			{
			if (pMat->isMat())
				{
				// Calculate histogram.  Currently defaults to grayscale.
				cv::Mat matHst;

				// Calculate histogram to new matrix
				cv::calcHist ( pMat->mat, 1, 0, cv::Mat(), matHst, 1, 
									&histSize, &histRange );

				// Copy to result
				matHst.copyTo(*(pMatHst->mat));
				}	// else
			#ifdef	HAVE_OPENCV_UMAT
			else if (pMat->isUMat())
				{
				cv::Mat	matHst,matLog;

				// 'calcHist' not available for UMat ?
				matHst = pMat->umat->getMat(cv::ACCESS_READ);

				// Histogram
				cv::calcHist ( &matHst, 1, 0, cv::noArray(), 
									matHst, 1, &histSize, &histRange );

				// Result
				matHst.copyTo ( *(pMat->umat) );
				}	// else if
			#endif
			#ifdef	HAVE_OPENCV_CUDA
			else if (pMat->isGPU())
				{
				// Calculate histogram.  Currently defaults to grayscale.
				cv::cuda::GpuMat	matHst;

				// Histogram
				cv::cuda::calcHist ( *(pMat->gpumat), matHst );

				// Copy to result
				matHst.copyTo(*(pMatHst->gpumat));
				}	// if
			#endif
			}	// if

		// Debug
		if (hr != S_OK)
			lprintf ( LOG_DBG, L"histogram failed : 0x%x\r\n", hr );

		// Result
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(pImgHst));
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pMat);
		_RELEASE(pImg);
		_RELEASE(pMatHst);
		_RELEASE(pImgHst);
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

