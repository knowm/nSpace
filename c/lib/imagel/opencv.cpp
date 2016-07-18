////////////////////////////////////////////////////////////////////////
//
//									OPENCV.CPP
//
//		OpenCV implmenetation of needed image processing functionality.
//
////////////////////////////////////////////////////////////////////////

#include "imagel_.h"
#include <stdio.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#if		_MSC_VER >= 1900
#else
//#include <opencv2/gpu/gpu.hpp>
#endif
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/features2d/features2d.hpp>

// Globals
using namespace cv;
static bool bInit = false;
static bool bGPU	= false;

// String references
static adtString	strRefWidth(L"Width");
static adtString	strRefHeight(L"Height");
static adtString	strRefBits(L"Bits");
static adtString	strRefFormat(L"Format");

#if	CV_MAJOR_VERSION == 3
HRESULT image_fft ( cv::UMat *pMat, cv::UMat *pWnd, bool bRows, bool bZeroDC )
#else
HRESULT image_fft ( cv::Mat *pMat, cv::Mat *pWnd, bool bRows, bool bZeroDC )
#endif
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Computes the FFT of an image.  Replaces image with FFT.
	//
	//	PARAMETERS
	//		-	pMat contains the image data.
	//		-	pWnd is an optional window function (NULL for none)
	//		-	bRows is true if image FFT should be row by row rather than
	//			full 2D FFT.
	//		-	bZeroDC is true to zero the DC component of the FFT
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr	= S_OK;
//	Mat		matPad;
	#if	CV_MAJOR_VERSION == 3
	UMat		matPad,matDft,matPlanes[2],matCmplx,matQ[4];
	UMat		matTmp,matMag,matReal;
	#else
	Mat		matPad,matDft,matPlanes[2],matCmplx,matQ[4];
	Mat		matTmp,matMag,matReal;
	#endif
	int		m,n,cx,cy;

	// Open CV uses exceptions
	try
		{
		// Apply window function
		if (	pWnd != NULL && pWnd->type() == pMat->type() &&
				pWnd->cols == pMat->cols && pWnd->rows == pMat->rows)
			multiply ( *pMat, *pWnd, *pMat );

		// Create a windowed version of the image
		m = getOptimalDFTSize ( pMat->rows );
		n = getOptimalDFTSize ( pMat->cols );
		copyMakeBorder ( (*pMat), matPad, 0, m - pMat->rows, 0, n - pMat->cols, 
								BORDER_CONSTANT, Scalar::all(0) );

		// Produce a real and (zeroed) imaginary pair
		#if	CV_MAJOR_VERSION == 3
		matPlanes[0].setTo(matPad);
		matPlanes[1].setTo(Mat::zeros ( matPad.size(), CV_32F ));
		#error	Not yet implemented
		#else
		matPlanes[0] = Mat_<float>(matPad);
		matPlanes[1] = Mat::zeros ( matPad.size(), CV_32F );		
		merge ( matPlanes, 2, matCmplx );
		#endif
	
		// Compute DFT
		// TODO: Option for scaling/inverse/magnitude
		dft ( matCmplx, matCmplx, (bRows) ? DFT_ROWS|DFT_SCALE : DFT_SCALE );

		// Separate real/imaginary results
		#if	CV_MAJOR_VERSION == 2
		split ( matCmplx, matPlanes );
		#endif

		// Compute the magnitude of DFT
		magnitude ( matPlanes[0], matPlanes[1], matPlanes[0] );
		matMag = matPlanes[0];
	
		// TODO: Log, base, etc. will be moved into own nodes.

		// Ensure no log of zeroes
		add ( matMag, Scalar::all(1e-20), matMag );

		// Log scale
		log ( matMag, matMag );

		// Need to take 20*log10(x)
		// Open CV log is natural log so scale for log10 (2.303)
		multiply ( matMag, Scalar::all(20.0/2.303), matMag );

		// Crop the spectrum if it has an odd number of rows or columns
		matMag = matMag ( Rect ( 0, 0, matMag.cols & -2, matMag.rows & -2 ) );

		// Rearrange the quadrants of Fourier image so that the origin is at the image center
		cx = matMag.cols/2;
		cy = matMag.rows/2;

		// ROIs for quadrants
		matQ[0] = matMag ( Rect ( 0, 0, cx, cy ) );
		matQ[1] = matMag ( Rect ( cx, 0, cx, cy ) );
		matQ[2] = matMag ( Rect ( 0, cy, cx, cy ) );
		matQ[3] = matMag ( Rect ( cx, cy, cx, cy ) );

		// Swap quadrants
		matQ[0].copyTo ( matTmp );
		matQ[1].copyTo ( matQ[0] );
		matTmp.copyTo  ( matQ[1] );

		matQ[2].copyTo ( matTmp );
		matQ[3].copyTo ( matQ[2] );
		matTmp.copyTo  ( matQ[3] );

		// Just keep the positive frequencies (option ?)
		matMag	= matMag ( Rect ( cx, 0, cx, matMag.rows ) );
		/*
		// DEBUG
		FILE *f = NULL;
		fopen_s ( &f, "c:/temp/data.txt", "w" );
		if (f != NULL)
			{
			for (int r = 0;r < matMag.rows;++r)
				for (int c = 0;c < matMag.cols;++c)
					fprintf ( f, "%d, %g\r\n", r*matMag.cols+c, matMag.at<float>(Point(c,r)) );
			fclose(f);
			}	// if
		*/

		// Zero the DC component on request
		if (bZeroDC)
			for (int r = 0;r < matMag.rows;++r)
				#if	CV_MAJOR_VERSION == 3
				matMag.getMat(cv::ACCESS_WRITE).at<float>(Point(0,r)) = 0.0f;
				#else
				matMag.at<float>(Point(0,r)) = 0.0f;
				#endif

		// Result is new matrix
		CCLOK ( matMag.copyTo ( *pMat ); )
		}	// try
	catch ( cv::Exception ex )
		{
		const char *c = ex.msg.c_str();
		dbgprintf ( L"image_fft:OpenCV exception\r\n" );
//		dbgprintf ( L"image_fft:OpenCV exception:%S\r\n", ex.msg.c_str() );
		}	// catch

	return hr;
	}	// image_fft
/*
HRESULT image_fft ( cv::ocl::oclMat *pMat, bool bRows, bool bZeroDC )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Computes the FFT of an image.  Replaces image with FFT.
	//			(OpenCL version)
	//
	//	PARAMETERS
	//		-	pMat contains the image data.
	//		-	bRows is true if image FFT should be row by row rather than
	//			full 2D FFT.
	//		-	bZeroDC is true to zero the DC component of the FFT
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr	= S_OK;
	ocl::oclMat	matPad,matDft,matPlanes[2],matCmplx,matQ[4];
	ocl::oclMat	matTmp,matMag,matReal;
	int			m,n,cx,cy;

	// Open CV uses exceptions
	try
		{
		// Convert source image to 32-bit floating point to match logic below
		pMat->convertTo ( matTmp, CV_32FC1 );

		// Create a windowed version of the image
		m = getOptimalDFTSize ( matTmp.rows );
		n = getOptimalDFTSize ( matTmp.cols );
		ocl::copyMakeBorder (	matTmp, matPad, 0, m - matTmp.rows, 0, n - matTmp.cols, 
										BORDER_CONSTANT, Scalar::all(0) );

		// Produce a real and (zeroed) imaginary pair
		matPlanes[0] = Mat_<float>(matPad);
		matPlanes[1] = Mat::zeros ( matPad.size(), CV_32F );
		ocl::merge ( matPlanes, 2, matCmplx );

		// Compute DFT
		// TODO: Option for scaling/inverse
		ocl::dft ( matCmplx, matCmplx, cv::Size(n,m),  
						(bRows) ? DFT_ROWS|DFT_SCALE : DFT_SCALE );

		// Separate real/imaginary results
		ocl::split ( matCmplx, matPlanes );

		// Compute the magnitude of DFT
		ocl::magnitude ( matPlanes[0], matPlanes[1], matPlanes[0] );
		matMag = matPlanes[0];
	
		// TODO: Log, base, etc. will be moved into own nodes.

		// Ensure no log of zeroes
		matMag += Scalar::all(1e-20);

		// Log scale
		ocl::log ( matMag, matMag );

		// Need to take 20*log10(x)
		// Open CV log is natural log so scale for log10 (2.303)
		matMag = (20.0/2.303) * matMag;

		// Crop the spectrum if it has an odd number of rows or columns
		matMag = matMag ( Rect ( 0, 0, matMag.cols & -2, matMag.rows & -2 ) );

		// Rearrange the quadrants of Fourier image so that the origin is at the image center
		cx = matMag.cols/2;
		cy = matMag.rows/2;

		// ROIs for quadrants
		matQ[0] = matMag ( Rect ( 0, 0, cx, cy ) );
		matQ[1] = matMag ( Rect ( cx, 0, cx, cy ) );
		matQ[2] = matMag ( Rect ( 0, cy, cx, cy ) );
		matQ[3] = matMag ( Rect ( cx, cy, cx, cy ) );

		// Swap quadrants
		matQ[0].copyTo ( matTmp );
		matQ[1].copyTo ( matQ[0] );
		matTmp.copyTo  ( matQ[1] );

		matQ[2].copyTo ( matTmp );
		matQ[3].copyTo ( matQ[2] );
		matTmp.copyTo  ( matQ[3] );

		// Just keep the positive frequencies (option ?)
		matMag	= matMag ( Rect ( cx, 0, cx, matMag.rows ) );

		// Zero the DC component on request
//		if (bZeroDC)
//			for (int r = 0;r < matMag.rows;++r)
//				matMag.at<float>(Point(0,r)) = 0.0f;

		// TODO: TEMPORARY.  This thresholding will be moved into a node
		// in dB
		CCLOK ( ocl::threshold ( matMag, matMag, 50, 0, THRESH_TRUNC ); )
		CCLOK ( ocl::threshold ( matMag, matMag, 15, 0, THRESH_TOZERO ); )

		// Result is new matrix
		CCLOK ( matMag.copyTo ( *pMat ); )
		}	// try
	catch ( cv::Exception ex )
		{
		lprintf ( LOG_ERR, L"OpenCV exception\r\n" );
		}	// catch

	return hr;
	}	// image_fft
*/

HRESULT image_load ( const WCHAR *pwLoc, IDictionary *pImg )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Load an image from disk.
	//
	//	PARAMETERS
	//		-	pwLoc is the source path/filename.
	//		-	pImg will receive the image data.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
//	Mat			*pmImg	= NULL;
	char			*paLoc	= NULL;
	adtString	strLoc(pwLoc);

	// OpenCV needs ASCII
	CCLTRY ( strLoc.toAscii ( &paLoc ) );

	// 'imread' does not seem to work, OpenCV is so flaky
	//	TODO: Replace with own PNG,BMP,etc persistence
//	cv::Mat
//	cv::imre
//	CCLTRYE ( (plImg = cvLoadImage ( paLoc, CV_LOAD_IMAGE_UNCHANGED )) != NULL, 
//					E_UNEXPECTED );

	// Convert image into dictionary
	if (hr == S_OK)
		{
		#if	CV_MAJOR_VERSION == 3
		cv::UMat mat = cv::imread ( paLoc, CV_LOAD_IMAGE_UNCHANGED ).getUMat(cv::ACCESS_READ);
//		cv::imread ( paLoc, CV_LOAD_IMAGE_UNCHANGED ).copyTo ( 
		hr = image_from_mat ( &mat, pImg );

//		cv::Mat mat = cv::imread ( paLoc, CV_LOAD_IMAGE_UNCHANGED );
		#elif	CV_MAJOR_VERSION == 2
		IplImage		*plImg = NULL;

		// Attempt load
		CCLTRYE ( (plImg	= cvLoadImage ( paLoc, CV_LOAD_IMAGE_UNCHANGED )) != NULL,
						E_UNEXPECTED );

		// Wrap into matrix
		if (hr == S_OK)
			{
			cv::Mat mat ( plImg, false );
			hr = image_from_mat ( &mat, pImg );
			}	// if

		// Clean up
		if (plImg != NULL)
			cvReleaseImage(&plImg);
		#endif

		}	// if

	// Clean up
	_FREEMEM(paLoc);

	return hr;
	}	// image_load

HRESULT image_save ( IDictionary *pImg, const WCHAR *pwLoc )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Save an image to disk.
	//
	//	PARAMETERS
	//		-	pImg contains the image data.
	//		-	pwLoc is the destination path/filename.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	#if	CV_MAJOR_VERSION == 3
	UMat			*pmImg	= NULL;
	#else
	Mat			*pmImg	= NULL;
	#endif
	char			*paLoc	= NULL;
	adtString	strLoc(pwLoc);

	// Convert image to OpenCV
	CCLTRY ( image_to_mat ( pImg, &pmImg ) );

	// OpenCV needs ASCII
	CCLTRY ( strLoc.toAscii ( &paLoc ) );

	// DEBUG
//	CCLOK ( invert ( *pmImg, *pmImg ); )
//	CCLOK ( bitwise_not ( *pmImg, *pmImg ); )

	#if	CV_MAJOR_VERSION == 3
	CCLOK ( cvSaveImage ( paLoc, &(IplImage(pmImg->getMat(cv::ACCESS_READ))) ); )
	#else
	// 'imwrite' does not seem to work, OpenCV is so flaky
	//	TODO: Replace with own PNG,BMP,etc persistence
	CCLOK ( cvSaveImage ( paLoc, &(IplImage(*pmImg)) ); )
	#endif

	// Clean up
	_FREEMEM(paLoc);
	if (pmImg != NULL)
		delete pmImg;

	return hr;
	}	// image_save

#if	CV_MAJOR_VERSION == 3
HRESULT image_from_mat ( UMat *pM, IDictionary *pImg )
#else
HRESULT image_from_mat ( Mat *pM, IDictionary *pImg )
#endif
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Convert an OpenCV mat object to an image dictionary.
	//
	//	PARAMETERS
	//		-	pM contains the image data
	//		-	pImg will receive the image data
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr			= S_OK;
	IMemoryMapped	*pBits	= NULL;
	VOID				*pvBits	= NULL;

	// Image information
	CCLTRY ( pImg->store ( strRefWidth, adtInt(pM->cols) ) );
	CCLTRY ( pImg->store ( strRefHeight, adtInt(pM->rows) ) );

	// Create and size memory block for image
	CCLTRY ( COCREATE ( L"Io.MemoryBlock", IID_IMemoryMapped, &pBits ) );
	CCLTRY ( pBits->setSize ( (U32)(pM->total()*pM->elemSize()) ) );
	CCLTRY ( pImg->store ( strRefBits, adtIUnknown(pBits) ) );
	CCLTRY ( pBits->lock ( 0, 0, &pvBits, NULL ) );

	// Copy bits
	#if	CV_MAJOR_VERSION == 3
	CCLOK  ( memcpy ( pvBits, pM->getMat(cv::ACCESS_READ).data, 
							pM->total()*pM->elemSize() ); )
	#else
	CCLOK  ( memcpy ( pvBits, pM->data, pM->total()*pM->elemSize() ); )
	#endif

	// Assign format, add new formats as needed.
	if (hr == S_OK)
		{
		switch ( pM->channels() )
			{
			case 1:
				// Gray-scale
				switch ( (pM->type() & CV_MAT_DEPTH_MASK) )
					{
					case CV_8U:
						CCLTRY ( pImg->store ( strRefFormat, adtString(L"U8x2") ) );
						break;
					case CV_16U:
						CCLTRY ( pImg->store ( strRefFormat, adtString(L"U16x2") ) );
						break;
					case CV_16S:
						CCLTRY ( pImg->store ( strRefFormat, adtString(L"S16x2") ) );
						break;
					case CV_32F:
						CCLTRY ( pImg->store ( strRefFormat, adtString(L"F32x2") ) );
						break;
					default :
						hr = E_NOTIMPL;
					}	// switch
				break;
	//		case 3 :
	//			if (pM->depth() == 8)
	//				hr = pImg->store ( strRefFormat, adtString(L"U8x2") );
	//			break;
			default :
				hr = E_NOTIMPL;
			}	// switch
		}	// if

	// Clean up
	_UNLOCK(pBits,pvBits);
	_RELEASE(pBits);

	return hr;
	}	// image_from_mat

#if	CV_MAJOR_VERSION == 3
HRESULT image_to_mat ( IDictionary *pImg, UMat **ppM )
#else
HRESULT image_to_mat ( IDictionary *pImg, Mat **ppM )
#endif
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Wrap an image inside an OpenCV matrix.
	//
	//	PARAMETERS
	//		-	pImg contains the image data.
	//		-	ppM will receive the matrix.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr			= S_OK;
	IMemoryMapped	*pBits	= NULL;
	VOID				*pvBits	= NULL;
	adtValue			vL;
	adtString		strFmt;
	adtIUnknown		unkV;
	U32				w,h,cvFmt;

	// Setup
	(*ppM) = NULL;

	// Access image information
	CCLTRY ( pImg->load ( strRefWidth, vL ) );
	CCLOK  ( w = adtInt(vL); )
	CCLTRY ( pImg->load ( strRefHeight, vL ) );
	CCLOK  ( h = adtInt(vL); )
	CCLTRY ( pImg->load ( strRefFormat, vL ) );
	CCLTRYE( (strFmt = vL).length() > 0, E_UNEXPECTED );
	CCLTRY ( pImg->load ( strRefBits, vL ) );
	CCLTRY ( _QISAFE((unkV=vL),IID_IMemoryMapped,&pBits) );
	CCLTRY ( pBits->lock ( 0, 0, &pvBits, NULL ) );

	// NOTE: Add needed formats over time
	if (hr == S_OK && !WCASECMP(strFmt,L"U16X2"))
		cvFmt = CV_16UC1;
	else if (hr == S_OK && !WCASECMP(strFmt,L"S16X2"))
		cvFmt = CV_16SC1;
	else if (hr == S_OK && !WCASECMP(strFmt,L"U8X2"))
		cvFmt = CV_8UC1;
	else if (hr == S_OK && !WCASECMP(strFmt,L"S8X2"))
		cvFmt = CV_8SC1;

	// Image formats
	else if (hr == S_OK && !WCASECMP(strFmt,L"B8G8R8"))
		cvFmt = CV_8UC3;
	else 
		hr = ERROR_NOT_SUPPORTED;

	#if	CV_MAJOR_VERSION == 3
	CCLTRYE ( ((*ppM) = new UMat ( h, w, cvFmt )) != NULL, E_OUTOFMEMORY );
	CCLOK ( cv::Mat ( h, w, cvFmt, pvBits ).copyTo ( *(*ppM) ); )
//	if (hr == S_OK)
//		{
//		cv::Mat mat ( h, w, cvFmt, pvBits );
//		(*ppM)->setTo ( mat );
//		}	// if
	#else
	CCLTRYE ( ((*ppM) = new Mat ( h, w, cvFmt, pvBits )) != NULL,
												E_OUTOFMEMORY );
	#endif

	// Clean up
	_UNLOCK(pBits,pvBits);
	_RELEASE(pBits);

	return hr;
	}	// image_to_mat
