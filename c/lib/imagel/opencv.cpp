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
#include <opencv2/gpu/gpu.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/features2d/features2d.hpp>

// Globals
using namespace cv;
static bool bInit = false;
static bool bGPU	= false;

HRESULT image_fft ( IDictionary *pImg, bool bZeroDC )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Computes the FFT of an image.  Replaces image with FFT.
	//
	//	PARAMETERS
	//		-	pImg contains the image data.
	//		-	bZeroDC is true to zero the DC component of the FFT
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

	// Open CV.  Make GPU more general.
	Mat			*pmImg	= NULL;
	Mat			matPad,matDft,matPlanes[2],matCmplx,matQ[4],matTmp,matMag,matReal;
	gpu::GpuMat	gpuPad,gpuDft,gpuPlanes[2],gpuCmplx,gpuQ[4],gpuTmp,gpuMag,gpuReal;
	int			m,n,cx,cy;

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

	// GPU enable ?
	if (hr == S_OK && !bInit)
		{
		bGPU = (gpu::getCudaEnabledDeviceCount() > 0);
		bInit = true;
		}	// if

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
		else if (hr == S_OK && !WCASECMP(strFmt,L"S16X2"))
			{
			CCLTRYE ( (pmImg = new Mat ( h, w, CV_16SC1, pvBits )) != NULL,
							E_OUTOFMEMORY );
			CCLOK   ( bpp = 2; )
			}	// if
		else 
			hr = ERROR_NOT_SUPPORTED;

		// Convert source image to 32-bit floating point to match logic below
		pmImg->convertTo ( matTmp, CV_32FC1 );
		if (bGPU)
			gpuTmp.upload ( matTmp );

		// Compute DFT (make "rows" an option)
//		dft ( matTmp, matReal, DFT_REAL_OUTPUT|DFT_ROWS );
//		dft ( matTmp, matReal, DFT_REAL_OUTPUT );

		// Switch to log scale
//		matReal += Scalar::all(1);
//		log ( matReal, matReal );

		// Create a windowed version of the image
		m = getOptimalDFTSize ( matTmp.rows );
		n = getOptimalDFTSize ( matTmp.cols );
		if (!bGPU)
			copyMakeBorder ( matTmp, matPad, 0, m - matTmp.rows, 0, n - matTmp.cols, 
									BORDER_CONSTANT, Scalar::all(0) );
		else 
			gpu::copyMakeBorder ( gpuTmp, gpuPad, 0, m - gpuTmp.rows, 0, n - gpuTmp.cols, 
											BORDER_CONSTANT, Scalar::all(0) );

		// Produce a real and (zeroed) imaginary pair
		matPlanes[0] = Mat_<float>(matPad);
		matPlanes[1] = Mat::zeros ( matPad.size(), CV_32F );
		if (!bGPU)
			merge ( matPlanes, 2, matCmplx );
		else
			{
			gpuPlanes[0].upload ( matPlanes[0] );
			gpuPlanes[1].upload ( matPlanes[1] );
			gpu::merge ( gpuPlanes, 2, gpuCmplx );
			}	// else

		// Compute DFT
		if (!bGPU)
			dft ( matCmplx, matCmplx, DFT_ROWS );
		else
			gpu::dft ( gpuCmplx, gpuCmplx, gpuCmplx.size(), DFT_ROWS );

//		dft ( matCmplx, matCmplx, 0 );

		// Separate real/imaginary results
		if (!bGPU)
			split ( matCmplx, matPlanes );
		else
			gpu::split ( gpuCmplx, gpuPlanes );

		// Compute the magnitude of DFT
		if (!bGPU)
			{
			magnitude ( matPlanes[0], matPlanes[1], matPlanes[0] );
			matMag = matPlanes[0];
			}	// if
		else
			{
			magnitude ( gpuPlanes[0], gpuPlanes[1], gpuPlanes[0] );
			gpuMag = gpuPlanes[0];
			}	// else
	
		// Switch to log scale
		if (!bGPU)
			{
			// Log scale
			matMag += Scalar::all(1);
			log ( matMag, matMag );

			// Crop the spectrum if it has an odd number of rows or columns
			matMag = matMag ( Rect ( 0, 0, matMag.cols & -2, matMag.rows & -2 ) );
			}	// if
		else
			{
			// Log scale
			gpu::add ( gpuMag, Scalar::all(1), gpuMag );
			gpu::log ( gpuMag, gpuMag );

			// Crop the spectrum if it has an odd number of rows or columns
			gpuMag = gpuMag ( Rect ( 0, 0, gpuMag.cols & -2, gpuMag.rows & -2 ) );
			}	// else

		// Rearrange the quadrants of Fourier image so that the origin is at the image center
		if (!bGPU)
			{
			cx = matMag.cols/2;
			cy = matMag.rows/2;
			}	// if
		else
			{
			cx = gpuMag.cols/2;
			cy = gpuMag.rows/2;
			}	// else

		// In OpenCV FFT example, however do not currently need all 4 quadrants.
		// Just use matQ[2] which contains the positive going frequency components.

		// ROIs for quadrants
//		matQ[0] = matMag ( Rect ( 0, 0, cx, cy ) );
//		matQ[1] = matMag ( Rect ( cx, 0, cx, cy ) );
		if (!bGPU)
			matQ[2] = matMag ( Rect ( 0, cy, cx, cy ) );
		else
			gpuQ[2] = gpuMag ( Rect ( 0, cy, cx, cy ) );
//		matQ[3] = matMag ( Rect ( cx, cy, cx, cy ) );

		// Swap quadrants
/*		matQ[0].copyTo ( matTmp );
		matQ[3].copyTo ( matQ[0] );
		matTmp.copyTo  ( matQ[3] );

		matQ[1].copyTo ( matTmp );
		matQ[2].copyTo ( matQ[1] );
		matTmp.copyTo  ( matQ[2] );
*/
		if (!bGPU)
			matMag	= matQ[2];
		else
			gpuMag	= gpuQ[2];

		// Zero the DC component on request
		if (bZeroDC)
			{
			// Middle column of each row = zero
//			for (int r = 0;r < matMag.rows;++r)
//				matMag.at<float>(Point(cx,r)) = 0.0f;
			// First column of each row = zero
			if (!bGPU)
				{
				for (int r = 0;r < matMag.rows;++r)
					matMag.at<float>(Point(0,r)) = 0.0f;
				}	// if

			// Last row of each column = zero
//			for (int c = 0;c < cx;++c)
//				matMag.at<float>(Point(c,cy-1)) = 0.0f;
			}	// if

		// Done with original image object
		delete pmImg;
		pmImg = NULL;

		// Ensure original bits have room for the possibly grown magnitude
		_UNLOCK(pBits,pvBits);
		CCLTRY(pBits->setSize ( (!bGPU) ?	matMag.rows*matMag.cols*bpp : 
														gpuMag.rows*gpuMag.cols*bpp ));
		CCLTRY(pBits->lock ( 0, 0, &pvBits, NULL ));
		CCLOK ( w = (!bGPU) ? matMag.cols : gpuMag.cols; )
		CCLOK ( h = (!bGPU) ? matMag.rows : gpuMag.rows; )

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
		if (hr == S_OK && (!WCASECMP(strFmt,L"U16X2") || !WCASECMP(strFmt,L"S16X2")))
			{
			// New image object for destination
			CCLTRYE ( (pmImg = new Mat ( h, w, CV_16UC1, pvBits )) != NULL,
							E_OUTOFMEMORY );

			// Normalize, convert, and copy
//			matReal.convertTo ( *pmImg, CV_16UC1 );
//			matReal.copyTo ( *pmImg );
			if (!bGPU)
				{
				normalize ( matMag, matMag, 0, 0xffff, NORM_MINMAX );
				matMag.convertTo ( *pmImg, CV_16UC1 );
				}	// if
			else
				{
				gpu::normalize ( gpuMag, gpuMag, 0, 0xffff, NORM_MINMAX );
				gpuMag.convertTo ( gpuMag, CV_16UC1 );
				gpuMag.download ( *pmImg );
				}	// if

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


