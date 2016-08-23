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
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pImg		= NULL;
	pWnd		= NULL;
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
	cv::Mat	matPad,matDft,matPlanes[2],matCmplx,matQ[4];
	cv::Mat	matTmp,matMag,matReal;
	int		m,n,cx,cy;

	// Open CV uses exceptions
	try
		{
		// Apply window function
		if (	pWnd != NULL && pWnd->type() == pMat->type() &&
				pWnd->cols == pMat->cols && pWnd->rows == pMat->rows)
			cv::multiply ( *pMat, *pWnd, *pMat );

		// Create a windowed version of the image
		m = cv::getOptimalDFTSize ( pMat->rows );
		n = cv::getOptimalDFTSize ( pMat->cols );
		cv::copyMakeBorder ( (*pMat), matPad, 0, m - pMat->rows, 0, n - pMat->cols, 
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

		// Rearrange the quadrants of Fourier image so that the origin is at the image center
		cx = matMag.cols/2;
		cy = matMag.rows/2;

		// ROIs for quadrants
//		matQ[0] = matMag ( cv::Rect ( 0, 0, cx, cy ) );
//		matQ[1] = matMag ( cv::Rect ( cx, 0, cx, cy ) );
//		matQ[2] = matMag ( cv::Rect ( 0, cy, cx, cy ) );
//		matQ[3] = matMag ( cv::Rect ( cx, cy, cx, cy ) );

		// Swap quadrants
//		matQ[0].copyTo ( matTmp );
//		matQ[1].copyTo ( matQ[0] );
//		matTmp.copyTo  ( matQ[1] );

//		matQ[2].copyTo ( matTmp );
//		matQ[3].copyTo ( matQ[2] );
//		matTmp.copyTo  ( matQ[3] );

		// Just keep the positive frequencies (option ?)
		matMag	= matMag ( cv::Rect ( 0, 0, cx, matMag.rows ) );

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
	cv::UMat	matPad,matDft,matCmplx,matQ[4];
	cv::UMat	matTmp,matMag,matReal;
	std::vector<cv::UMat>	
				matPlanes(2);
	int		m,n,cx,cy;

	// Open CV uses exceptions
	try
		{
		// Apply window function
		if (	pWnd != NULL && pWnd->type() == pMat->type() &&
				pWnd->cols == pMat->cols && pWnd->rows == pMat->rows)
			cv::multiply ( *pMat, *pWnd, *pMat );

		// Create a windowed version of the image
		m = cv::getOptimalDFTSize ( pMat->rows );
		n = cv::getOptimalDFTSize ( pMat->cols );
		cv::copyMakeBorder ( (*pMat), matPad, 0, m - pMat->rows, 0, n - pMat->cols, 
									cv::BORDER_CONSTANT, cv::Scalar::all(0) );

		// Produce a real and (zeroed) imaginary pair
		matPlanes[0] = matPad;
		matPlanes[1] = cv::UMat ( matPad.size(), CV_32F );
		matPlanes[1].setTo ( cv::Scalar(0) );
		cv::merge ( matPlanes, matCmplx );

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

		// Rearrange the quadrants of Fourier image so that the origin is at the image center
		cx = matMag.cols/2;
		cy = matMag.rows/2;

		// ROIs for quadrants
//		matQ[0] = matMag ( cv::Rect ( 0, 0, cx, cy ) );
//		matQ[1] = matMag ( cv::Rect ( cx, 0, cx, cy ) );
//		matQ[2] = matMag ( cv::Rect ( 0, cy, cx, cy ) );
//		matQ[3] = matMag ( cv::Rect ( cx, cy, cx, cy ) );

		// Swap quadrants
//		matQ[0].copyTo ( matTmp );
//		matQ[1].copyTo ( matQ[0] );					// CRASH! On Intel Iris
//		matTmp.copyTo  ( matQ[1] );

//		matQ[2].copyTo ( matTmp );
//		matQ[3].copyTo ( matQ[2] );
//		matTmp.copyTo  ( matQ[3] );

		// Just keep the positive frequencies (option ?)
		matMag	= matMag ( cv::Rect ( 0, 0, cx, matMag.rows ) );

		// Result is new matrix
		matMag.copyTo ( *pMat );
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
	cv::cuda::GpuMat	matPad,matDft,matPlanes[2],matCmplx,matQ[4];
	cv::cuda::GpuMat	matTmp,matMag,matReal;
	int					m,n,cx,cy;
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

		// Create a windowed version of the image
		m = cv::getOptimalDFTSize ( pMat->rows );
		n = cv::getOptimalDFTSize ( pMat->cols );
		cv::cuda::copyMakeBorder ( (*pMat), matPad, 0, m - pMat->rows, 0, n - pMat->cols, 
											cv::BORDER_CONSTANT, cv::Scalar::all(0) );

		// Debug
//		QueryPerformanceCounter ( &lCnt );
//		dt = ((lCnt.QuadPart-lRst.QuadPart) * 1.0) / lFreq.QuadPart;
//		lprintf ( LOG_DBG, L"fft (GPU) : %g", dt );

		// Produce a real and (zeroed) imaginary pair
		Convert::convertTo ( &matPad, &matPlanes[0], CV_32F );
		matPlanes[1] = cv::cuda::GpuMat ( matPad.size(), CV_32F );
		matPlanes[1].setTo ( cv::Scalar(0) );
		cv::cuda::merge ( matPlanes, 2, matCmplx );

		// Debug
//		QueryPerformanceCounter ( &lCnt );
//		dt = ((lCnt.QuadPart-lRst.QuadPart) * 1.0) / lFreq.QuadPart;
//		lprintf ( LOG_DBG, L"fft (GPU) : %g", dt );

		// Compute DFT
		cv::cuda::dft ( matCmplx, matCmplx, matCmplx.size(), (bRows) ? cv::DFT_ROWS : 0 );

		// Debug
//		QueryPerformanceCounter ( &lCnt );
//		dt = ((lCnt.QuadPart-lRst.QuadPart) * 1.0) / lFreq.QuadPart;
//		lprintf ( LOG_DBG, L"fft (GPU) : %g", dt );

		// Separate real/imaginary results
		cv::cuda::split ( matCmplx, matPlanes );

		// Compute the magnitude of DFT
		cv::cuda::magnitude ( matPlanes[0], matPlanes[1], matPlanes[0] );
		matMag = matPlanes[0];

		// TODO: Log, base, etc. will be moved into own nodes.

		// Ensure no log of zeroes
		cv::cuda::add ( matMag, cv::Scalar::all(1e-20), matMag );

		// Log scale
		cv::cuda::log ( matMag, matMag );

		// Need to take 20*log10(x)
		// Open CV log is natural log so scale for log10 (2.303)
		cv::cuda::multiply ( matMag, cv::Scalar::all(20.0/2.303), matMag );

		// Crop the spectrum if it has an odd number of rows or columns
		matMag = matMag ( cv::Rect ( 0, 0, matMag.cols & -2, matMag.rows & -2 ) );

		// Rearrange the quadrants of Fourier image so that the origin is at the image center
		cx = matMag.cols/2;
		cy = matMag.rows/2;

		// ROIs for quadrants
//		matQ[0] = matMag ( cv::Rect ( 0, 0, cx, cy ) );
//		matQ[1] = matMag ( cv::Rect ( cx, 0, cx, cy ) );
//		matQ[2] = matMag ( cv::Rect ( 0, cy, cx, cy ) );
//		matQ[3] = matMag ( cv::Rect ( cx, cy, cx, cy ) );

		// Swap quadrants
//		matQ[0].copyTo ( matTmp );
//		matQ[1].copyTo ( matQ[0] );
//		matTmp.copyTo  ( matQ[1] );

//		matQ[2].copyTo ( matTmp );
//		matQ[3].copyTo ( matQ[2] );
//		matTmp.copyTo  ( matQ[3] );

		// Just keep the positive frequencies (option ?)
		matMag	= matMag ( cv::Rect ( 0, 0, cx, matMag.rows ) );

		// Result is new matrix
		matMag.copyTo ( *pMat );
		}	// try
	catch ( cv::Exception &ex )
		{
		lprintf ( LOG_WARN, L"fft:cv::cuda:GpuMat:exception:%S\r\n",
						ex.err.c_str() );
		hr = E_UNEXPECTED;
		}	// catch

	return hr;
	}	// fft

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
		if (hr == S_OK && pMat->isGPU())
			hr = fft ( pMat->gpumat, (pWnd != NULL) ? pWnd->gpumat : NULL, true );
		else if (hr == S_OK && pMat->isUMat())
			hr = fft ( pMat->umat, (pWnd != NULL) ? pWnd->umat : NULL, true );
		else
			hr = fft ( pMat->mat, (pWnd != NULL) ? pWnd->mat : NULL, true );

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
	if (pMat->isGPU())
		{
		rows = pMat->gpumat->rows;
		cols = pMat->gpumat->cols;
		}	// if
	else if (pMat->isUMat())
		{
		rows = pMat->umat->rows;
		cols = pMat->umat->cols;
		}	// if
	else
		{
		rows = pMat->mat->rows;
		cols = pMat->mat->cols;
		}	// if
	if (pWnd != NULL)
		{
		if (pWnd->isGPU())
			{
			wrows = pWnd->gpumat->rows;
			wcols = pWnd->gpumat->cols;
			}	// if
		else if (pWnd->isUMat())
			{
			wrows = pWnd->umat->rows;
			wcols = pWnd->umat->cols;
			}	// else if
		else
			{
			wrows = pWnd->mat->rows;
			wcols = pWnd->mat->cols;
			}	// else
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
		if (pWnd->isGPU())
			pWnd->gpumat->upload ( matWnd );
		else if (pWnd->isUMat())
			matWnd.copyTo ( *(pWnd->umat) );
		else
			matWnd.copyTo ( *(pWnd->mat) );
		}	// if

	return hr;
	}	// window

