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

	// Using OpenCV, needs full, valid Windows path
	#ifdef	_WIN32
	CCLOK ( strLoc.replace ( '/', '\\' ); )
	#endif

	// OpenCV needs ASCII
	CCLTRY ( strLoc.toAscii ( &paLoc ) );

	// 'imread' does not seem as stable on OpenCV.
	//	TODO: Replace with own PNG,BMP,etc persistence

	// Convert image into dictionary
	if (hr == S_OK)
		{
//		cv::UMat mat = cv::imread ( paLoc, CV_LOAD_IMAGE_UNCHANGED ).getUMat(cv::ACCESS_READ);
//		hr = image_from_mat ( &mat, pImg );
		IplImage		*plImg = NULL;

		// Attempt load
		CCLTRYE ( (plImg	= cvLoadImage ( paLoc, CV_LOAD_IMAGE_UNCHANGED )) != NULL,
						E_UNEXPECTED );

		// Wrap into matrix
		if (hr == S_OK)
			{
//			cv::Mat mat ( plImg, false );
			cv::Mat mat = cv::cvarrToMat(plImg);
			hr = image_from_mat ( &mat, pImg );
			}	// if

		// Clean up
		if (plImg != NULL)
			cvReleaseImage(&plImg);
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
	Mat			*pmImg	= NULL;
	char			*paLoc	= NULL;
	adtString	strLoc(pwLoc);
	int			ret		= 0;
	bool			b			= false;

	// Using OpenCV, needs full, valid Windows path
	#ifdef	_WIN32
	CCLOK ( strLoc.replace ( '/', '\\' ); )
	#endif

	// Convert image to OpenCV
	CCLTRY ( image_to_mat ( pImg, &pmImg ) );

	// OpenCV needs ASCII
	CCLTRY ( strLoc.toAscii ( &paLoc ) );

	// 'imwrite' does not seem to work, OpenCV is so flaky
	//	TODO: Replace with own PNG,BMP,etc persistence
//	CCLOK ( ret = cvSaveImage ( paLoc, &(IplImage(*pmImg)) ); )
	CCLOK ( b = imwrite ( paLoc, *pmImg ); )
	if (!b)
		lprintf ( LOG_INFO, L"cvSaveImage : %S : b %d errno %d\r\n", 
										paLoc, b, errno );

	// Clean up
	_FREEMEM(paLoc);
	if (pmImg != NULL)
		delete pmImg;

	return hr;
	}	// image_save

HRESULT image_to_debug ( cvMatRef *pMat, const WCHAR *pwCtx, 
									const WCHAR *pwLoc )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Output/save debug information about an image.
	//
	//	PARAMETERS
	//		-	pMat is the matrix object
	//		-	pwCtx is the context
	//		-	pwLoc is the destination for the image
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	char			*paLoc	= NULL;
	U32			type		= 0;
	cv::Point	ptMin,ptMax;
	double		dMin,dMax;
	adtString	strLoc(pwLoc);
	cv::Mat		matSave;

	// Values and locations of min and max
	if (pMat->isUMat())
		{
		cv::minMaxLoc ( *(pMat->umat), &dMin, &dMax, &ptMin, &ptMax );
		type = pMat->umat->type();
		}	// if
	#ifdef	HAVE_OPENCV_CUDA
	else if (pMat->isGPU())
		{
		cv::cuda::minMaxLoc ( *(pMat->gpumat), &dMin, &dMax, &ptMin, &ptMax );
		type = pMat->gpumat->type();
		}	// if
	#endif
	else
		{
		cv::minMaxLoc ( *(pMat->mat), &dMin, &dMax, &ptMin, &ptMax );
		type = pMat->mat->type();
		}	// if
	dbgprintf ( L"%s (%s) : Min : %g @ (%d,%d) : Max : %g @ (%d,%d) : type %d\r\n",
						pwCtx, 
						(pMat->isGPU()) ? L"GPU" :
						(pMat->isUMat()) ? L"UMAT" : L"CPU",
						dMin, ptMin.x, ptMin.y, dMax, ptMax.x, ptMax.y,
						type );

	// Download
	if (pMat->isUMat())
		pMat->umat->copyTo ( matSave );
	#ifdef	HAVE_OPENCV_CUDA
	else if (pMat->isGPU())
		pMat->gpumat->download ( matSave );
	#endif
	else
		pMat->mat->copyTo ( matSave );

	// Some sample point values
	if (matSave.type() == CV_32FC1)
		dbgprintf ( L"%g %g %g %g - %g %g %g %g\r\n",
						matSave.at<float>(0,0),
						matSave.at<float>(0,1),
						matSave.at<float>(0,2),
						matSave.at<float>(0,3),
						matSave.at<float>(1,0),
						matSave.at<float>(1,1),
						matSave.at<float>(1,2),
						matSave.at<float>(1,3) );

	// Conversion for saving to image
	cv::normalize ( matSave, matSave, 0, 255, cv::NORM_MINMAX );
	matSave.convertTo ( matSave, CV_8UC1 );

	// Save to file
	if (strLoc.toAscii ( &paLoc ) == S_OK)
		cv::imwrite ( paLoc, matSave );
	
	// Clean up
	_FREEMEM(paLoc);

	return hr;
	}	// image_to_debug

HRESULT image_from_mat ( Mat *pM, IDictionary *pImg )
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
	CCLOK  ( memcpy ( pvBits, pM->data, pM->total()*pM->elemSize() ); )

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
			case 3 :
				// Color
				switch ( (pM->type() & CV_MAT_DEPTH_MASK) )
					{
					case CV_8U :
						CCLTRY ( pImg->store ( strRefFormat, adtString(L"B8G8R8") ) );
						break;
					default :
						hr = E_NOTIMPL;
					}	// switch
				break;
			default :
				lprintf ( LOG_WARN, L"Unsupported channels %d", pM->channels() );
				hr = E_NOTIMPL;
			}	// switch
		}	// if

	// Debug
	if (hr != S_OK)
		lprintf ( LOG_WARN, L"Failed 0x%x", hr );

	// Clean up
	_UNLOCK(pBits,pvBits);
	_RELEASE(pBits);

	return hr;
	}	// image_from_mat

HRESULT image_to_mat ( IDictionary *pImg, Mat **ppM )
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
	else if (hr == S_OK && !WCASECMP(strFmt,L"F32X2"))
		cvFmt = CV_32FC1;

	// Image formats
	else if (hr == S_OK && 
				(!WCASECMP(strFmt,L"R8G8B8") || !WCASECMP(strFmt,L"B8G8R8")))
		cvFmt = CV_8UC3;
	else 
		hr = ERROR_NOT_SUPPORTED;

	// Wrap bits
	CCLTRYE ( ((*ppM) = new Mat ( h, w, cvFmt, pvBits )) != NULL,
												E_OUTOFMEMORY );

	// Clean up
	_UNLOCK(pBits,pvBits);
	_RELEASE(pBits);

	return hr;
	}	// image_to_mat
