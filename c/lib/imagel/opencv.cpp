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
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/features2d/features2d.hpp>

// Globals
using namespace cv;

HRESULT image_fft ( IDictionary *pImg )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Computes the FFT of an image.  Replaces image with FFT.
	//
	//	PARAMETERS
	//		-	pImg contains the image data.
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
	U32				w,h,bpp	= 0;

	// Open CV
	Mat	*pmImg	= NULL;
	Mat	matPad,matDft,matPlanes[2],matCmplx,matQ[4],matTmp,matMag,matReal;
	int	m,n,cx,cy;

	// Access image information
	CCLTRY ( pImg->load ( adtString(L"Width"), vL ) );
	CCLOK  ( w = adtInt(vL); )
	CCLTRY ( pImg->load ( adtString(L"Height"), vL ) );
	CCLOK  ( h = adtInt(vL); )
	CCLTRY ( pImg->load ( adtString(L"Format"), vL ) );
	CCLTRYE( (strFmt = vL).length() > 0, E_UNEXPECTED );
	CCLTRY ( pImg->load ( adtString(L"Bits"), vL ) );
	CCLTRY ( _QISAFE((unkV=vL),IID_IMemoryMapped,&pBits) );
	CCLTRY ( pBits->lock ( 0, 0, &pvBits, NULL ) );

	// Open CV uses exceptions
	try
		{
		// NOTE: Add needed formats over time
		if (hr == S_OK && !WCASECMP(strFmt,L"U16X2"))
			{
			CCLTRYE ( (pmImg = new Mat ( h, w, CV_16UC1, pvBits )) != NULL,
							E_OUTOFMEMORY );
			CCLOK   ( bpp = 2; )
			}	// if
		else 
			hr = ERROR_NOT_SUPPORTED;

		// Convert source image to 32-bit floating point to match logic below
		pmImg->convertTo ( matTmp, CV_32FC1 );

		// Compute DFT (make "rows" an option)
//		dft ( matTmp, matReal, DFT_REAL_OUTPUT|DFT_ROWS );
//		dft ( matTmp, matReal, DFT_REAL_OUTPUT );

		// Switch to log scale
//		matReal += Scalar::all(1);
//		log ( matReal, matReal );

		// Create a windowed version of the image
		m = getOptimalDFTSize ( matTmp.rows );
		n = getOptimalDFTSize ( matTmp.cols );
		copyMakeBorder ( matTmp, matPad, 0, m - matTmp.rows, 0, n - matTmp.cols, 
								BORDER_CONSTANT, Scalar::all(0) );

		// Produce a real and (zeroed) imaginary pair
		matPlanes[0] = Mat_<float>(matPad);
		matPlanes[1] = Mat::zeros ( matPad.size(), CV_32F );
		merge ( matPlanes, 2, matCmplx );

		// Compute DFT
		dft ( matCmplx, matCmplx, DFT_ROWS );

		// Separate real/imaginary results
		split ( matCmplx, matPlanes );

		// Compute the magnitude of DFT
		magnitude ( matPlanes[0], matPlanes[1], matPlanes[0] );
		matMag = matPlanes[0];
	
		// Switch to log scale
		matMag += Scalar::all(1);
		log ( matMag, matMag );

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
		matQ[3].copyTo ( matQ[0] );
		matTmp.copyTo  ( matQ[3] );

		matQ[1].copyTo ( matTmp );
		matQ[2].copyTo ( matQ[1] );
		matTmp.copyTo  ( matQ[2] );

		// Done with original image object
		delete pmImg;
		pmImg = NULL;

		// Ensure original bits have room for the possibly grown magnitude
		_UNLOCK(pBits,pvBits);
		CCLTRY(pBits->setSize ( matMag.rows*matMag.cols*bpp ));
		CCLTRY(pBits->lock ( 0, 0, &pvBits, NULL ));
		CCLOK ( w = matMag.cols; )
		CCLOK ( h = matMag.rows; )

		// Update descriptor
		CCLTRY ( pImg->store ( adtString(L"Width"), adtInt(w) ) );
		CCLTRY ( pImg->store ( adtString(L"Height"), adtInt(h) ) );

		// DEBUG.  For now zero the DC component.

		// DEBUG
/*		static bool bFirst = true;
		if (bFirst)
			{
			bFirst = false;
			FILE *f = NULL;
			fopen_s ( &f, "c:\\temp\\mag.txt", "w" );
			for (U32 i = 0;i < w;++i)
				fprintf ( f, "%d, %g\r\n", i, matMag.at<float>(Point(i,10)) );
			fclose(f);
			}	// if
*/

		// For now, convert back into the original format.  
		// This requires normalization and conversion
		// NOTE: Add needed formats over time
		if (hr == S_OK && !WCASECMP(strFmt,L"U16X2"))
			{
			// New image object for destination
			CCLTRYE ( (pmImg = new Mat ( h, w, CV_16UC1, pvBits )) != NULL,
							E_OUTOFMEMORY );

			// Normalize, convert, and copy
//			matReal.convertTo ( *pmImg, CV_16UC1 );
//			matReal.copyTo ( *pmImg );
			normalize ( matMag, matMag, 0, 0xffff, NORM_MINMAX );
			matMag.convertTo ( *pmImg, CV_16UC1 );
//			matMag.copyTo ( *pmImg );
			}	// if
		else 
			hr = ERROR_NOT_SUPPORTED;
		}	// try
	catch ( cv::Exception ex )
		{
		dbgprintf ( L"image_fft:OpenCV exception:%S\r\n", ex.msg.c_str() );
		}	// catch

	// Clean up
	if (pmImg != NULL)
		delete pmImg;
	_UNLOCK(pBits,pvBits);
	_RELEASE(pBits);

	return hr;
	}	// image_fft


