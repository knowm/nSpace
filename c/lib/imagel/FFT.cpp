////////////////////////////////////////////////////////////////////////
//
//									FFT.CPP
//
//				Implementation of the image FFT node.
//
////////////////////////////////////////////////////////////////////////

#define	INITGUID
#include "imagel_.h"
#include <stdio.h>

// Globals

FFT :: FFT ( void )
	#ifdef	HAVE_OPENCV_UMAT
	: umatPlanes(2)
	#endif
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pImg		= NULL;
	pWnd		= NULL;
	bRows		= false;
	}	// FFT

HRESULT FFT :: fft ( cv::Mat *pMat, cv::Mat *pWnd, bool bRows )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	cv::Mat based FFT.
	//
	//	PARAMETERS
	//		-	pMat contains the image data.
	//		-	pWnd is an optional window function (NULL for none)
	//		-	bRows is true if image FFT should be row by row rather than
	//			full 2D FFT.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr	= S_OK;
	cv::Mat	matPad,matDft,matPlanes[2],matCmplx,matMag;
	int		m,n,cx,cy,orows,ocols;

	// Open CV uses exceptions
	try
		{
		// Apply window function
		if (	pWnd != NULL && pWnd->type() == pMat->type() &&
				pWnd->cols == pMat->cols && pWnd->rows == pMat->rows)
			cv::multiply ( *pMat, *pWnd, *pMat );

		// Original number of rows and columns
		orows = pMat->rows;
		ocols = pMat->cols;

		// Create a windowed version of the image
		m = cv::getOptimalDFTSize ( orows );
		n = cv::getOptimalDFTSize ( ocols );
		cv::copyMakeBorder ( (*pMat), matPad, 0, m - orows, 0, n - ocols, 
									cv::BORDER_CONSTANT, cv::Scalar::all(0) );

		// Produce a real and (zeroed) imaginary pair
		matPlanes[0] = cv::Mat_<float>(matPad);
		matPlanes[1] = cv::Mat::zeros ( matPad.size(), CV_32F );		
		cv::merge ( matPlanes, 2, matCmplx );
	
		// Compute DFT
		cv::dft ( matCmplx, matCmplx, (bRows) ? cv::DFT_ROWS : 0 );

		// Separate real/imaginary results
		cv::split ( matCmplx, matPlanes );

		// Compute the magnitude of DFT
		cv::magnitude ( matPlanes[0], matPlanes[1], matPlanes[0] );
		matMag = matPlanes[0];

		// Normalize by the number of samples (convention ?)
		if (bRows)
			cv::divide ( matMag, cv::Scalar(matMag.cols), matMag );
		else
			cv::divide ( matMag, cv::Scalar(matMag.cols*matMag.rows), matMag );
	
		// TODO: Log, base, etc. will be moved into own nodes.

		// Ensure no log of zeroes
		cv::add ( matMag, cv::Scalar::all(1e-20), matMag );

		// Log scale
		cv::log ( matMag, matMag );

		// Need to take 20*log10(x)
		// Open CV log is natural log so scale for log10 (2.303)
		cv::multiply ( matMag, cv::Scalar::all(20.0/2.303), matMag );

		// Crop the spectrum if it has an odd number of rows or columns
		matMag = matMag ( cv::Rect ( 0, 0, matMag.cols & -2, matMag.rows & -2 ) );

		// Keep a single quadrant
		cx = matMag.cols/2;
		cy = matMag.rows/2;

		// Do not include any extra rows/cols that might have been introduced
		// in the padding.  Just keep the positive frequencies (option ?)
		if (bRows)
			matMag	= matMag ( cv::Rect ( 0, 0, cx, (orows < matMag.rows) ? orows : matMag.rows ) );
		else
			matMag	= matMag ( cv::Rect ( 0, 0,	(ocols < cx) ? ocols : cx, 
																(orows < matMag.rows) ? orows : matMag.rows ) );

		// Result is new matrix
		matMag.copyTo ( *pMat );
		}	// try
	catch ( cv::Exception & )
		{
		lprintf ( LOG_WARN, L"fft:cv::Mat:exception\r\n" );
		hr = E_UNEXPECTED;
//		dbgprintf ( L"image_fft:OpenCV exception:%S\r\n", ex.msg.c_str() );
		}	// catch

	return hr;
	}	// fft

#ifdef	HAVE_OPENCV_UMAT
HRESULT FFT :: fft ( cv::UMat *pMat, cv::UMat *pWnd, bool bRows )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	cv::Umat based FFT.
	//
	//	PARAMETERS
	//		-	pMat contains the image data.
	//		-	pWnd is an optional window function (NULL for none)
	//		-	bRows is true if image FFT should be row by row rather than
	//			full 2D FFT.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr	= S_OK;
	int		m,n,cx,cy,or,oc;

	// Open CV uses exceptions
	try
		{
		// Apply window function
		if (	pWnd != NULL && pWnd->type() == pMat->type() &&
				pWnd->cols == pMat->cols && pWnd->rows == pMat->rows)
			cv::multiply ( *pMat, *pWnd, *pMat );

		// Original number of rows and columns
		or = pMat->rows;
		oc = pMat->cols;

		// Create a windowed version of the image
		m = cv::getOptimalDFTSize ( pMat->rows );
		n = cv::getOptimalDFTSize ( pMat->cols );
		cv::copyMakeBorder ( (*pMat), umatPad, 0, m - pMat->rows, 0, n - pMat->cols, 
									cv::BORDER_CONSTANT, cv::Scalar::all(0) );

		// Produce a real and (zeroed) imaginary pair
		Convert::convertTo ( &umatPad, &umatPlanes[0], CV_32F );
//		umatPlanes[0] = umatPad;
		umatPlanes[1] = cv::UMat ( umatPad.size(), CV_32F );
		umatPlanes[1].setTo ( cv::Scalar(0) );
		cv::merge ( umatPlanes, umatCmplx );

		// Compute DFT
		cv::dft ( umatCmplx, umatCmplx, (bRows) ? cv::DFT_ROWS : 0 );

		// Separate real/imaginary results
		cv::split ( umatCmplx, umatPlanes );

		// Compute the magnitude of DFT
		cv::magnitude ( umatPlanes[0], umatPlanes[1], umatPlanes[0] );
		umatMag = umatPlanes[0];

		// Normalize by the number of samples (convention ?)
		if (bRows)
			cv::divide ( umatMag, cv::Scalar(umatMag.cols), umatMag );
		else
			cv::divide ( umatMag, cv::Scalar(umatMag.cols*umatMag.rows), umatMag );
	
		// TODO: Log, base, etc. will be moved into own nodes.

		// Ensure no log of zeroes
		cv::add ( umatMag, cv::Scalar::all(1e-20), umatMag );

		// Log scale
		cv::log ( umatMag, umatMag );

		// Need to take 20*log10(x)
		// Open CV log is natural log so scale for log10 (2.303)
		cv::multiply ( umatMag, cv::Scalar::all(20.0/2.303), umatMag );

		// Crop the spectrum if it has an odd number of rows or columns
		umatMag = umatMag ( cv::Rect ( 0, 0, umatMag.cols & -2, umatMag.rows & -2 ) );

		// Keep a single quadrant
		cx = umatMag.cols/2;
		cy = umatMag.rows/2;

		// Do not include any extra rows/cols that might have been introduced
		// in the padding.  Just keep the positive frequencies (option ?)
		if (bRows)
			umatMag	= umatMag ( cv::Rect ( 0, 0, cx, (or < umatMag.rows) ? or : umatMag.rows ) );
		else
			umatMag	= umatMag ( cv::Rect ( 0, 0,	(oc < cx) ? oc : cx, 
																(or < umatMag.rows) ? or : umatMag.rows ) );

		// Just keep the positive frequencies (option ?)
//		umatMag	= umatMag ( cv::Rect ( 0, 0, cx, umatMag.rows ) );

		// Result is new matrix
		umatMag.copyTo ( *pMat );
		}	// try
	catch ( cv::Exception &ex )
		{
		lprintf ( LOG_WARN, L"fft:cv::UMat:exception:%S\r\n",
						ex.err.c_str() );
		hr = E_UNEXPECTED;
//		dbgprintf ( L"image_fft:OpenCV exception:%S\r\n", ex.msg.c_str() );
		}	// catch

	return hr;
	}	// fft
#endif

#ifdef HAVE_OPENCV_CUDA
HRESULT FFT :: fft ( cv::cuda::GpuMat *pMat, cv::cuda::GpuMat *pWnd, 
							bool bRows )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	cv::cuda::GpuMat based FFT.
	//
	//	PARAMETERS
	//		-	pMat contains the image data.
	//		-	pWnd is an optional window function (NULL for none)
	//		-	bRows is true if image FFT should be row by row rather than
	//			full 2D FFT.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT				hr	= S_OK;
	int					m,n,cx,cy,or,oc;

	// Open CV uses exceptions
	try
		{
		// Timing debug
//		LARGE_INTEGER	lCnt,lRst,lFreq;
//		double			dt;
//		QueryPerformanceCounter ( &lRst );
//		QueryPerformanceFrequency ( &lFreq );

		// Apply window function
		if (	pWnd != NULL && pWnd->type() == pMat->type() &&
				pWnd->cols == pMat->cols && pWnd->rows == pMat->rows)
			cv::cuda::multiply ( *pMat, *pWnd, *pMat );

		// Debug
//		QueryPerformanceCounter ( &lCnt );
//		dt = ((lCnt.QuadPart-lRst.QuadPart) * 1.0) / lFreq.QuadPart;
//		lprintf ( LOG_DBG, L"fft (GPU) : %g", dt );

		// Original number of rows and columns
		or = pMat->rows;
		oc = pMat->cols;

		// Create a windowed version of the image
		m = cv::getOptimalDFTSize ( pMat->rows );
		n = cv::getOptimalDFTSize ( pMat->cols );
		cv::cuda::copyMakeBorder ( (*pMat), gpuPad, 0, m - pMat->rows, 0, n - pMat->cols, 
											cv::BORDER_CONSTANT, cv::Scalar::all(0) );

		// Debug
//		QueryPerformanceCounter ( &lCnt );
//		dt = ((lCnt.QuadPart-lRst.QuadPart) * 1.0) / lFreq.QuadPart;
//		lprintf ( LOG_DBG, L"fft (GPU) : %g", dt );

		// Produce a real and (zeroed) imaginary pair
		Convert::convertTo ( &gpuPad, &gpuPlanes[0], CV_32F );
		gpuPlanes[1] = cv::cuda::GpuMat ( gpuPad.size(), CV_32F );
		gpuPlanes[1].setTo ( cv::Scalar(0) );
		cv::cuda::merge ( gpuPlanes, 2, gpuCmplx );

		// Debug
//		QueryPerformanceCounter ( &lCnt );
//		dt = ((lCnt.QuadPart-lRst.QuadPart) * 1.0) / lFreq.QuadPart;
//		lprintf ( LOG_DBG, L"fft (GPU) : %g", dt );

		// Compute DFT
		cv::cuda::dft ( gpuCmplx, gpuCmplx, gpuCmplx.size(), (bRows) ? cv::DFT_ROWS : 0 );

		// Debug
//		QueryPerformanceCounter ( &lCnt );
//		dt = ((lCnt.QuadPart-lRst.QuadPart) * 1.0) / lFreq.QuadPart;
//		lprintf ( LOG_DBG, L"fft (GPU) : %g", dt );

		// Separate real/imaginary results
		cv::cuda::split ( gpuCmplx, gpuPlanes );

		// Compute the magnitude of DFT
		cv::cuda::magnitude ( gpuPlanes[0], gpuPlanes[1], gpuPlanes[0] );
//		gpuMag = gpuPlanes[0];

		// Normalize by the number of samples (convention ?)
		if (bRows)
			cv::cuda::divide ( gpuPlanes[0], cv::Scalar(gpuPlanes[0].cols), gpuPlanes[0] );
		else
			cv::cuda::divide ( gpuPlanes[0], cv::Scalar(gpuPlanes[0].cols*gpuPlanes[0].rows), gpuPlanes[0] );

		// Ensure no log of zeroes
		cv::cuda::add ( gpuPlanes[0], cv::Scalar::all(1e-20), gpuPlanes[0] );

		// Log scale
		cv::cuda::log ( gpuPlanes[0], gpuPlanes[0] );

		// Need to take 20*log10(x)
		// Open CV log is natural log so scale for log10 (2.303)
		cv::cuda::multiply ( gpuPlanes[0], cv::Scalar::all(20.0/2.303), gpuPlanes[0] );

		// Crop the spectrum if it has an odd number of rows or columns
		gpuMag = gpuPlanes[0] ( cv::Rect ( 0, 0, gpuMag.cols & -2, gpuMag.rows & -2 ) );

		// Keep a single quadrant
		cx = gpuMag.cols/2;
		cy = gpuMag.rows/2;

		// Do not include any extra rows/cols that might have been introduced
		// in the padding.  Just keep the positive frequencies (option ?)
		if (bRows)
			gpuMag	= gpuMag ( cv::Rect ( 0, 0, cx, (or < gpuMag.rows) ? or : gpuMag.rows ) );
		else
			gpuMag	= gpuMag ( cv::Rect ( 0, 0,	(oc < cx) ? oc : cx, 
																(or < gpuMag.rows) ? or : gpuMag.rows ) );

		// Just keep the positive frequencies (option ?)
//		gpuMag	= gpuMag ( cv::Rect ( 0, 0, cx, gpuMag.rows ) );

		// Result is new matrix
		gpuMag.copyTo ( *pMat );
		}	// try
	catch ( cv::Exception &ex )
		{
		lprintf ( LOG_WARN, L"fft:cv::cuda:GpuMat:exception:%S\r\n",
						ex.err.c_str() );
		hr = E_UNEXPECTED;
		}	// catch

	return hr;
	}	// fft
#endif

HRESULT FFT :: onAttach ( bool bAttach )
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

		// Defaults
		if (pnDesc->load ( adtString(L"Window"), vL ) == S_OK)
			adtValue::toString ( vL, strWnd );
		if (pnDesc->load ( adtString(L"Rows"), vL ) == S_OK)
			bRows = vL;
		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pImg);
		if (pWnd != NULL)
			{
			delete pWnd;
			pWnd = NULL;
			}	// if
		}	// else

	return hr;
	}	// onAttach

HRESULT FFT :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

		// Windowing
		CCLTRY ( window(pMat) );

		// Compute FFT/Magnitude
//		CCLOK ( image_to_debug ( pMat, L"FFT", L"c:/temp/fft.png" ); )
		if (hr == S_OK && pMat->isMat())
			hr = fft ( pMat->mat, (pWnd != NULL) ? pWnd->mat : NULL, bRows );
		#ifdef	HAVE_OPENCV_UMAT
		else if (hr == S_OK && pMat->isUMat())
			hr = fft ( pMat->umat, (pWnd != NULL) ? pWnd->umat : NULL, bRows );
		#endif
		#ifdef HAVE_OPENCV_CUDA
		else if (hr == S_OK && pMat->isGPU())
			hr = fft ( pMat->gpumat, (pWnd != NULL) ? pWnd->gpumat : NULL, bRows );
		#endif

		// Debug
		if (hr != S_OK)
			lprintf ( LOG_WARN, L"FFT failed, hr 0x%x\r\n", hr );
//		CCLOK ( image_to_debug ( pMat, L"FFT", L"c:/temp/fft.png" ); )

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
	else if (_RCP(Window))
		{
		// New window name
		adtValue::toString ( v, strWnd );

		// Assume a new window is needed
		_RELEASE(pWnd);
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

HRESULT FFT :: window ( cvMatRef *pMat )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Verify the current window function is appropriate.
	//
	//	PARAMETERS
	//		-	pMat is the source image
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;
	bool		bWndUp	= false;
	int		rows,cols,wrows,wcols;

	//
	// Windowing.  Only generate a single row window function as
	// things change.  If the window function or number of columns
	// change it is time to re-create the window
	//

	// Sizing
	if (pMat->isMat())
		{
		rows = pMat->mat->rows;
		cols = pMat->mat->cols;
		}	// if
	#ifdef	HAVE_OPENCV_UMAT
	else if (pMat->isUMat())
		{
		rows = pMat->umat->rows;
		cols = pMat->umat->cols;
		}	// if
	#endif
	#ifdef	HAVE_OPENCV_CUDA
	else if (pMat->isGPU())
		{
		rows = pMat->gpumat->rows;
		cols = pMat->gpumat->cols;
		}	// if
	#endif
	if (pWnd != NULL)
		{
		if (pWnd->isMat())
			{
			wrows = pWnd->mat->rows;
			wcols = pWnd->mat->cols;
			}	// else
		#ifdef	HAVE_OPENCV_UMAT
		else if (pWnd->isUMat())
			{
			wrows = pWnd->umat->rows;
			wcols = pWnd->umat->cols;
			}	// else if
		#endif
		#ifdef	HAVE_OPENCV_CUDA
		else if (pWnd->isGPU())
			{
			wrows = pWnd->gpumat->rows;
			wcols = pWnd->gpumat->cols;
			}	// if
		#endif
		}	// if

	// No window exists but one is specified
	if (!bWndUp) bWndUp = (pWnd == NULL	&& strWnd.length() > 0 && WCASECMP(strWnd,L"None"));

	// Verify size
	if (!bWndUp) bWndUp = (pWnd != NULL && (wcols != cols || wrows != rows));

	// New window ?
	if (hr == S_OK && bWndUp)
		{
		// Will write to a local array first
		cv::Mat matWnd (rows,cols,CV_32FC1);

		// Previous window
		_RELEASE(pWnd);

		// Generate function
		if (!WCASECMP(strWnd,L"Blackman-Nutall"))
			{
			// Coefficients for window
			float a0 = 0.3635819f;
			float a1 = 0.4891775f;
			float a2 = 0.1365995f;
			float a3 = 0.0106441f;
			float ev = 0.0f;

			// Compute window (slow)
			for (int r = 0;r < rows;++r)
				for (int c = 0;c < cols;++c)
					{
					// Evaulate
					ev =	(float) (a0 -
										a1 * cos ( (2*CV_PI*c)/(cols-1) ) +
										a2 * cos ( (4*CV_PI*c)/(cols-1) ) -
										a3 * cos ( (6*CV_PI*c)/(cols-1) ) );

					// Set
					matWnd.at<float>(cv::Point(c,r)) = ev;

					// Debug
//						matWnd.at<float>(cv::Point(c,r)) = 0;
					}	// for

			}	// if

		// Unknown function
		else
			{
			lprintf ( LOG_ERR, L"Unimplemented window function : %s", (LPCWSTR) strWnd );
			hr = E_NOTIMPL;
			}	// else

		// Need to create window matrix of matching type
		CCLTRY ( Create::create ( NULL, cols, rows, CV_32FC1, &pWnd ) );

		// Transfer data to window object
		if (pWnd->isMat())
			matWnd.copyTo ( *(pWnd->mat) );
		#ifdef	HAVE_OPENCV_UMAT
		else if (pWnd->isUMat())
			matWnd.copyTo ( *(pWnd->umat) );
		#endif
		#ifdef	HAVE_OPENCV_CUDA
		else if (pWnd->isGPU())
			pWnd->gpumat->upload ( matWnd );
		#endif
		}	// if

	return hr;
	}	// window

